"""
Tests of the shared modules: parameters and relations (Section 2), the manifest
and its file-name rule (Section 9), the two-plane targets, ray casting and
geometric visibility (Sections 3, 12, 14), the plane utilities, and the
registration solves (Section 4).
"""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from sensorperf.features.planes import fit_plane_robust, normalized_height, plane_depth_image
from sensorperf.geometry.registration import (
    METHOD_DEPTH_PLANES, PlaneObservation, Registration, plane_of_pose, solve_from_planes, solve_hand_eye,
)
from sensorperf.geometry.targets import (
    FEATURE_CUTOUT, FEATURE_DISK, SQUARE_EDGES, SURFACE_BACK, SURFACE_FRONT, SURFACE_NONE, StereoGeometry,
    TargetSet, fronto_parallel_pose, geometric_visibility, load_targets_asbuilt, make_standard_target_set,
    tilted_pose,
)
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.manifest import (
    FrameRecord, format_file_name, group_by_pose, load_manifest, parse_file_name, select, write_manifest_csv,
)
from sensorperf.parameters import CharacterizationParameters, MissingSensorValue, SensorGeometry

PARAMS = CharacterizationParameters()
GEOMETRY = SensorGeometry.indicative()
TEST_DEPTH_MM = 750.0
TEST_GAP_MM = 60.0
HAND_EYE_POSES = 12
HAND_EYE_NOISE_MM = 0.05
"""Translation noise on the observed target poses in the noisy hand-eye test."""
HAND_EYE_TOLERANCE_MM = 0.1
"""Allowed camera_to_base translation error with that noise."""
PLANE_SOLVE_POSES = 12
PLANE_SOLVE_ROTATION_TOLERANCE_DEG = 0.01
Z_OFFSET_SE_POSES = 30
"""Registration poses of the camera Z offset standard error test."""
Z_OFFSET_SE_PLANE_NOISE_MM = 0.02
"""Plane-distance noise (one standard deviation) added to the observed planes in that test."""
Z_OFFSET_SE_LARGE_TILT_DEG = 20.0
Z_OFFSET_SE_SMALL_TILT_DEG = 5.0
"""Tilt ranges (+/- about both axes) compared in that test."""
Z_OFFSET_SE_FACTOR = 3.0
"""Allowed ratio between the observed standard error and the conditioning estimate, in either direction."""
Z_OFFSET_SE_DEPTH_RANGE_MM = (500.0, 1000.0)
Z_OFFSET_SE_LATERAL_RANGE_MM = 50.0
"""Plate distance range and lateral spread (half range) of the synthetic poses."""
RNG_SEED = 7


def test_relations_section_2():
    """Section 2: p(Z), D_px, d = k/Z and delta Z_q are consistent with each other."""
    z = 750.0
    p = GEOMETRY.pixel_footprint_mm(z)
    assert p == pytest.approx(z / GEOMETRY.sensor_fx_px)
    assert GEOMETRY.diameter_in_pixels(2.0 * p, z) == pytest.approx(2.0)
    k = GEOMETRY.disparity_constant_mm_px()
    q = 0.125
    dz = GEOMETRY.depth_quantum_mm(q, z)
    assert GEOMETRY.disparity_quantum_px(dz, z) == pytest.approx(q)
    assert dz == pytest.approx(q * z ** 2 / k)
    assert len(PARAMS.noise_stations_mm()) == 11
    assert PARAMS.rule_of_three_bound(300) == pytest.approx(1.0 - 0.05 ** (1.0 / 300.0))
    with pytest.raises(MissingSensorValue):
        SensorGeometry().pixel_footprint_mm(z)


def test_station_ladder_is_geometric_from_z_min_to_z_max():
    """Redesign note, Section 1: the one station ladder has four stations per octave from Z_MIN = 400 to
    Z_MAX = 1600 mm, rounded to 1 mm, nine stations; the shape and reduced stations are strides of it, and the A
    stations add the two legacy depths."""
    assert PARAMS.z_min_mm == 400.0 and PARAMS.z_max_mm == 1600.0 and PARAMS.z_reference_mm == 800.0
    assert PARAMS.z_stations_mm() == (400.0, 476.0, 566.0, 673.0, 800.0, 951.0, 1131.0, 1345.0, 1600.0)
    assert PARAMS.z_shape_stations_mm() == (400.0, 566.0, 800.0, 1131.0, 1600.0)
    assert PARAMS.z_reduced_stations_mm() == (400.0, 800.0, 1600.0)
    assert PARAMS.z_reference_mm in PARAMS.z_stations_mm()
    assert PARAMS.noise_stations_mm() == (400.0, 476.0, 566.0, 673.0, 700.0, 800.0, 951.0, 1000.0, 1131.0, 1345.0,
                                          1600.0)
    assert PARAMS.detection_low_stations_mm() == (1131.0, 1345.0, 1600.0)
    # Consecutive stations differ by the ratio 2^(1/4) up to the 1 mm rounding.
    ratios = np.array(PARAMS.z_stations_mm()[1:]) / np.array(PARAMS.z_stations_mm()[:-1])
    assert np.allclose(ratios, PARAMS.z_station_ratio, rtol=2.0e-3)


def test_feature_ladder_diameters_cover_three_to_ninety_six_pixels():
    """Redesign note, Section 2: D_k = 3 px x p(Z_MAX) x (2 sqrt 2)^k gives 7.0, 19.7 and 55.8 mm at the indicative
    geometry, covering 3.0 to 12, 8.5 to 34 and 24 to 96 px over Z_MIN to Z_MAX."""
    diameters = PARAMS.feature_diameters_mm(GEOMETRY)
    assert len(diameters) == PARAMS.feature_count == 3
    assert diameters == pytest.approx((7.0, 19.7, 55.8), abs=0.1)
    assert np.allclose(np.diff(np.log(diameters)), np.log(PARAMS.feature_ladder_ratio))
    px_far = [GEOMETRY.diameter_in_pixels(d, PARAMS.z_max_mm) for d in diameters]
    px_near = [GEOMETRY.diameter_in_pixels(d, PARAMS.z_min_mm) for d in diameters]
    assert px_far[0] == pytest.approx(PARAMS.feature_min_px_at_z_max)
    assert px_far == pytest.approx([3.0, 8.5, 24.0], abs=0.05) and px_near == pytest.approx([12.0, 34.0, 96.0], abs=0.1)


def test_standard_target_set_has_one_disk_plate_and_one_cutout_plate():
    """Redesign note, Section 2: T1 is gone; T4 (disks) and T5 (cutouts) each carry three features and three blank
    sites, blank site i sized to the search window of feature i at Z_MAX (about 21, 34 and 70 mm), the disk plate also
    one post-only site; the isolation is 30 px at Z_MAX between every pair of sites, and the plate fits the field of
    view at Z_MIN with room for the phase-jitter span and the boundary band."""
    targets = make_standard_target_set(PARAMS, GEOMETRY)
    assert set(targets.targets) == {"T2", "T3a", "T3b", "T4", "T5"}
    p_far = GEOMETRY.pixel_footprint_mm(PARAMS.z_max_mm)
    diameters = PARAMS.feature_diameters_mm(GEOMETRY)
    windows_mm = [d + 2.0 * PARAMS.detection_window_margin_px * p_far for d in diameters]
    assert windows_mm == pytest.approx([20.9, 33.7, 69.8], abs=0.1)
    for target_id, kind, posts in (("T4", "disk", PARAMS.post_sites_per_plate), ("T5", "cutout", 0)):
        target = targets.get(target_id)
        features = [f for f in target.features if f.kind == kind]
        blanks = [f for f in target.features if f.kind == "blank"]
        assert sorted(f.diameter_mm for f in features) == pytest.approx(sorted(diameters))
        assert len(blanks) == PARAMS.blank_sites_per_plate == 3
        # Blank site i serves feature i: its diameter is that feature's search window at Z_MAX.
        blanks = sorted(blanks, key=lambda b: b.level_index)
        assert [b.level_index for b in blanks] == [0, 1, 2]
        assert [b.diameter_mm for b in blanks] == pytest.approx(windows_mm)
        assert sum(f.kind == "post" for f in target.features) == posts
        # Edge-to-edge spacing of every pair of sites is at least the isolation of 30 px at Z_MAX.
        isolation = PARAMS.feature_isolation_px * p_far
        sites = [f for f in target.features if f.kind != "post"]
        for i, a in enumerate(sites):
            for b in sites[i + 1:]:
                centers = np.hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm)
                assert centers - (a.diameter_mm + b.diameter_mm) / 2.0 >= isolation - 1.0e-6
        # The plate fits the field at Z_MIN with the jitter span and a boundary band on every side, and holds its sites.
        p_near = GEOMETRY.pixel_footprint_mm(PARAMS.z_min_mm)
        reserve = (PARAMS.phase_jitter_span_px + 2.0 * PARAMS.boundary_band_half_width_px) * p_near
        field_w, field_h = (2.0 * h for h in GEOMETRY.half_field_mm(PARAMS.z_min_mm))
        assert 2.0 * target.half_width_mm <= field_w - reserve and 2.0 * target.half_height_mm <= field_h - reserve
        for f in target.features:
            assert abs(f.x_mm) + f.diameter_mm / 2.0 <= target.half_width_mm
            assert abs(f.y_mm) + f.diameter_mm / 2.0 <= target.half_height_mm


def test_standard_target_set_raises_when_a_plate_cannot_fit_the_field_at_z_min():
    """A ladder whose largest blank site alone is wider than the usable field width cannot be laid out: the target set
    raises an error that names the plate and the sizes (it is not just a planner warning)."""
    too_wide = replace(PARAMS, feature_min_px_at_z_max=6.0)          # features of 14, 39 and 112 mm
    with pytest.raises(ValueError, match=r"plate T4 is \d+ x \d+ mm but only \d+ x \d+ mm fit the field of view at Z_MIN"):
        make_standard_target_set(too_wide, GEOMETRY)


def test_file_name_rule_round_trip():
    """Section 9: the file-name rule parses what it formats."""
    name = format_file_name("C", "T5", 15.0, 800.0, 0, 17, 3)
    assert name == "C_T5_G15_Z0800_F0_P017_f03.mc"
    fields = parse_file_name(name)
    assert fields == {"procedure": "C", "target_id": "T5", "gap_mm": 15.0, "station_z_mm": 800.0, "field": 0,
                      "pose_index": 17, "frame_index": 3}
    assert parse_file_name(format_file_name("A", "T2", None, 400.0, 3, 1, 99))["gap_mm"] is None
    with pytest.raises(ValueError):
        parse_file_name("not_a_capture.mc")


def test_manifest_round_trip(tmp_path: Path):
    """Section 9: every manifest column survives a write and read, poses group by pose key."""
    pose = tilted_pose(10.0, -5.0, TEST_DEPTH_MM, "V", 15.0)
    records = [FrameRecord(tmp_path / format_file_name("A", "T2", None, TEST_DEPTH_MM, 0, 4, f), "A", "T2", None,
                           TEST_DEPTH_MM, 0, 4, f, RigidTransform.from_rotation_vector_degrees([1, 2, 3], [100, 200, 300]),
                           pose, seed=11, offset_h_mm=0.5, offset_v_mm=-0.25, timestamp="t",
                           sensor_temp_c=31.5, subseries="tilt", tilt_axis="V", tilt_deg=15.0, metadata={"note": "x"})
               for f in range(3)]
    path = write_manifest_csv(tmp_path / "manifest.csv", records)
    loaded = load_manifest(path)
    assert len(loaded) == 3
    first = loaded[0]
    assert first.pose_key() == records[0].pose_key()
    assert first.target_pose_camera.difference_from(pose) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert first.robot_pose.difference_from(records[0].robot_pose) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert first.tilt_axis == "V" and first.tilt_deg == 15.0 and first.metadata == {"note": "x"}
    assert list(group_by_pose(loaded)) == [records[0].pose_key()]
    assert len(select(loaded, procedure="A", subseries="tilt")) == 3
    assert select(loaded, procedure=("B", "C")) == []


def test_cutout_array_ray_cast_and_visibility():
    """Sections 3 and 12: rays through a cutout reach the back plate at Z + G, rays on the
    plate stop at Z, and part of the back plate seen through small holes is hidden from
    the right camera (V = 0) while the front face is always visible."""
    targets = make_standard_target_set(PARAMS, GEOMETRY)
    stereo = StereoGeometry.from_sensor_geometry(GEOMETRY)
    target = targets.get("T5", TEST_GAP_MM)
    pose = fronto_parallel_pose(0.0, 0.0, TEST_DEPTH_MM)
    hit = target.intersect_rays(pose, np.zeros(3), stereo.camera.ray_directions())
    front, back = hit.surface == SURFACE_FRONT, hit.surface == SURFACE_BACK
    assert front.any() and back.any() and (hit.surface == SURFACE_NONE).any()
    assert np.allclose(hit.point_camera[front][:, 2], TEST_DEPTH_MM)
    assert np.allclose(hit.point_camera[back][:, 2], TEST_DEPTH_MM + TEST_GAP_MM)
    visible = geometric_visibility(target, pose, hit.point_camera, stereo, require_projector=True)
    assert visible[front].all()
    assert 0.0 < visible[back].mean() < 1.0
    # The smallest cutouts hide more of their back plate than the largest.
    cutouts = sorted(target.features_of_kind(FEATURE_CUTOUT), key=lambda f: f.diameter_mm)
    xy = hit.xy_target

    def hidden_fraction(feature):
        inside = back & feature.inside(np.nan_to_num(xy[..., 0]), np.nan_to_num(xy[..., 1]))
        return 1.0 - visible[inside].mean() if inside.any() else float("nan")

    small, large = hidden_fraction(cutouts[0]), hidden_fraction(cutouts[-1])
    assert np.isnan(small) or small >= large


def test_signed_distance_sign_convention():
    """Sections 11 and 14: s is positive on the front-material side for every feature kind."""
    targets = make_standard_target_set(PARAMS, GEOMETRY)
    disk = targets.get("T4").features_of_kind(FEATURE_DISK)[0]
    cutout = targets.get("T5").features_of_kind(FEATURE_CUTOUT)[0]
    assert disk.signed_distance_mm(disk.x_mm, disk.y_mm) > 0.0            # disk center is front material
    assert cutout.signed_distance_mm(cutout.x_mm, cutout.y_mm) < 0.0      # hole center is back plate
    square = targets.get("T3a").features[0]
    assert all(square.signed_distance_mm(0.0, 0.0, edge) > 0.0 for edge in SQUARE_EDGES)
    window = targets.get("T3b").features[0]
    assert window.signed_distance_mm(0.0, 0.0) < 0.0
    far = square.diameter_mm                                                # a point well outside the square
    assert square.signed_distance_mm(far, 0.0) < 0.0 and window.signed_distance_mm(far, 0.0) > 0.0


def test_target_set_and_asbuilt_round_trip(tmp_path: Path):
    """Section 3.3: the as-built record overrides the nominal feature sizes."""
    targets = make_standard_target_set(PARAMS, GEOMETRY)
    targets.save(tmp_path / "targets.json")
    reloaded = TargetSet.load(tmp_path / "targets.json")
    assert set(reloaded.targets) == set(targets.targets)
    template = reloaded.write_asbuilt_template(tmp_path / "asbuilt.csv")
    rows = load_targets_asbuilt(template)
    row = next(r for r in rows if r["kind"] == FEATURE_CUTOUT)
    row["diameter_mm"] = row["diameter_mm"] * 1.1
    row["diameter_uncertainty_mm"] = 0.02
    reloaded.apply_asbuilt([row])
    feature = reloaded.get(row["target_id"]).feature(row["site_id"])
    assert feature.diameter_mm == pytest.approx(row["diameter_mm"])
    assert feature.diameter_uncertainty_mm == pytest.approx(0.02)


def test_plane_utilities():
    """Sections 10 and 11: plane depth images and normalized height behave on a tilted plane."""
    stereo = StereoGeometry.from_sensor_geometry(GEOMETRY)
    camera = stereo.camera
    pose = tilted_pose(0.0, 0.0, TEST_DEPTH_MM, "V", 20.0)
    targets = make_standard_target_set(PARAMS, GEOMETRY)
    plate = targets.get("T2")
    point, normal = plate.front_plane_camera(pose)
    depth = plane_depth_image(camera, point, normal)
    points = camera.back_project(*camera.pixel_grid(), depth)
    fit = fit_plane_robust(points[np.isfinite(depth)])
    assert abs(fit.normal @ normal) == pytest.approx(1.0, abs=1e-9)
    assert fit.rms_mm < 1e-6
    z_back = depth + TEST_GAP_MM
    h = normalized_height(depth, depth, z_back)
    assert np.allclose(h[np.isfinite(h)], 1.0)
    assert np.allclose(normalized_height(z_back, depth, z_back)[np.isfinite(h)], 0.0)


def _random_flange_poses(rng, count):
    return [RigidTransform.from_rotation_vector_degrees(rng.normal(0.0, 25.0, 3), rng.normal(0.0, 200.0, 3) + [800.0, 0.0, 500.0])
            for _ in range(count)]


def test_hand_eye_recovers_known_transforms(tmp_path: Path):
    """Section 4, Step 7: the hand-eye solve recovers X and Y exactly from exact poses and to
    within the noise level from noisy ones; registration.json round-trips."""
    rng = np.random.default_rng(RNG_SEED)
    x_true = RigidTransform.from_rotation_vector_degrees([1.0, 2.0, 3.0], [10.0, 20.0, 150.0])
    y_true = RigidTransform.from_rotation_vector_degrees([170.0, 10.0, -20.0], [800.0, -100.0, 600.0])
    flange = _random_flange_poses(rng, HAND_EYE_POSES)
    observed = [y_true.inverse().compose(a).compose(x_true) for a in flange]
    exact = solve_hand_eye(flange, observed, PARAMS.registration_residual_accept_mm)
    assert exact.residual_rms_mm < 1e-8 and exact.accepted
    assert exact.camera_to_base.difference_from(y_true)[0] < 1e-8
    noisy = [RigidTransform(b.rotation, b.translation + rng.normal(0.0, HAND_EYE_NOISE_MM, 3)) for b in observed]
    solved = solve_hand_eye(flange, noisy, PARAMS.registration_residual_accept_mm)
    assert solved.camera_to_base.difference_from(y_true)[0] < HAND_EYE_TOLERANCE_MM
    assert solved.target_to_camera(flange[0]).difference_from(observed[0])[0] < HAND_EYE_TOLERANCE_MM
    # Planning direction: the flange pose for a wanted target pose reproduces that target pose.
    wanted = fronto_parallel_pose(0.0, 0.0, TEST_DEPTH_MM)
    commanded = exact.flange_to_base_for(wanted)
    assert exact.target_to_camera(commanded).difference_from(wanted) == pytest.approx((0.0, 0.0), abs=1e-8)
    exact.save(tmp_path / "registration.json")
    loaded = Registration.load(tmp_path / "registration.json")
    assert loaded.camera_to_base.difference_from(exact.camera_to_base) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_plane_only_registration_recovers_rotation_and_scale():
    """Section 4, Step 6 (second bullet) and Section 15: from depth planes only, the rotation is
    recovered and the plane distances are reproduced, while the in-plane parts of X stay fixed."""
    rng = np.random.default_rng(RNG_SEED + 1)
    x_true = RigidTransform(np.eye(3), np.array([0.0, 0.0, 120.0]))
    y_true = RigidTransform.from_rotation_vector_degrees([175.0, 5.0, -10.0], [700.0, 50.0, 400.0])
    flange = _random_flange_poses(rng, PLANE_SOLVE_POSES)
    planes = [plane_of_pose(y_true.inverse().compose(a).compose(x_true)) for a in flange]
    solved = solve_from_planes(flange, planes, accept_rms_mm=PARAMS.registration_residual_accept_mm)
    assert solved.method == METHOD_DEPTH_PLANES
    assert solved.residual_rms_mm < 1e-6
    assert solved.camera_to_base.difference_from(y_true)[1] < PLANE_SOLVE_ROTATION_TOLERANCE_DEG


def _z_offset_standard_error_mm(tilt_range_deg: float, seed: int = RNG_SEED + 2) -> float:
    """camera_z_offset_se_mm of a plane-only solve of synthetic poses: Z_OFFSET_SE_POSES plate poses with tilts
    uniform over +/- tilt_range_deg about both axes, and plane distances with Gaussian noise."""
    rng = np.random.default_rng(seed)
    x_true = RigidTransform(np.eye(3), np.array([0.0, 0.0, 120.0]))
    y_true = RigidTransform.from_rotation_vector_degrees([175.0, 5.0, -10.0], [700.0, 50.0, 400.0])
    flange, planes = [], []
    for _ in range(Z_OFFSET_SE_POSES):
        tilt = np.radians(rng.uniform(-tilt_range_deg, tilt_range_deg, 2))
        position = [rng.uniform(-Z_OFFSET_SE_LATERAL_RANGE_MM, Z_OFFSET_SE_LATERAL_RANGE_MM),
                    rng.uniform(-Z_OFFSET_SE_LATERAL_RANGE_MM, Z_OFFSET_SE_LATERAL_RANGE_MM),
                    rng.uniform(*Z_OFFSET_SE_DEPTH_RANGE_MM)]
        target_to_camera = RigidTransform(Rotation.from_euler("xy", tilt).as_matrix(), np.array(position))
        flange.append(y_true.compose(target_to_camera).compose(x_true.inverse()))
        plane = plane_of_pose(target_to_camera)
        planes.append(PlaneObservation(plane.normal, plane.distance_mm + rng.normal(0.0, Z_OFFSET_SE_PLANE_NOISE_MM)))
    return solve_from_planes(flange, planes).camera_z_offset_se_mm


def test_camera_z_offset_standard_error_follows_the_tilt_conditioning(tmp_path: Path):
    """Section 4, Step 4.7: the standard error of the camera's Z offset (the camera_to_base translation along the mean
    plate normal) comes from the Gauss-Newton fit covariance. Plane distances condition that offset only through the
    tilt range: a plate tilted by theta changes the distance along the normal by the factor cos(theta), so the offset
    is known to about sigma / ((1 - cos(theta)) sqrt(N)) for N poses, plane-distance noise sigma and tilts up to
    theta. With 30 poses, sigma = 0.02 mm and +/- 20 degrees about both axes that is 0.02 / (1 - cos 20 deg) /
    sqrt(30), about 0.06 mm; the solve must land within a factor of 3 of it, and the +/- 5 degree range (conditioning
    1 / (1 - cos 5 deg), 14 times worse) must give a larger standard error. The value is saved in registration.json."""
    large = _z_offset_standard_error_mm(Z_OFFSET_SE_LARGE_TILT_DEG)
    small = _z_offset_standard_error_mm(Z_OFFSET_SE_SMALL_TILT_DEG)
    print(f"camera Z offset standard error: {large:.4f} mm at +/-{Z_OFFSET_SE_LARGE_TILT_DEG:g} deg, "
          f"{small:.4f} mm at +/-{Z_OFFSET_SE_SMALL_TILT_DEG:g} deg")
    expected = Z_OFFSET_SE_PLANE_NOISE_MM / (1.0 - np.cos(np.radians(Z_OFFSET_SE_LARGE_TILT_DEG))) \
        / np.sqrt(Z_OFFSET_SE_POSES)
    assert np.isfinite(large) and large > 0.0
    assert expected / Z_OFFSET_SE_FACTOR < large < expected * Z_OFFSET_SE_FACTOR
    assert np.isfinite(small) and small > large
    # The standard error is part of registration.json and round-trips.
    registration = Registration(RigidTransform.identity(), RigidTransform.identity(), camera_z_offset_se_mm=large)
    registration.save(tmp_path / "registration.json")
    assert Registration.load(tmp_path / "registration.json").camera_z_offset_se_mm == pytest.approx(large)
