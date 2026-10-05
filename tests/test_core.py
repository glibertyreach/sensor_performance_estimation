"""
Tests of the shared modules: parameters and relations (Section 2), the manifest
and its file-name rule (Section 9), the two-plane targets, ray casting and
geometric visibility (Sections 3, 12, 14), the plane utilities, and the
registration solves (Section 4).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from sensorperf.features.planes import fit_plane_robust, normalized_height, plane_depth_image
from sensorperf.geometry.registration import (
    METHOD_DEPTH_PLANES, Registration, plane_of_pose, solve_from_planes, solve_hand_eye,
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


def test_diameter_ladder_spans_the_footprints():
    """Section 3.2: the ladder runs from 0.3 p(Z_MIN) to at least 30 p(Z_MAX) in steps of sqrt 2."""
    ladder = PARAMS.diameter_ladder_mm(GEOMETRY)
    assert ladder[0] == pytest.approx(PARAMS.diameter_min_footprint_fraction * GEOMETRY.pixel_footprint_mm(PARAMS.z_min_mm))
    assert ladder[-1] >= PARAMS.diameter_max_footprint_multiple * GEOMETRY.pixel_footprint_mm(PARAMS.z_max_mm)
    ratios = np.diff(np.log(ladder))
    assert np.allclose(ratios, np.log(PARAMS.diameter_ladder_ratio))


def test_file_name_rule_round_trip():
    """Section 9: the file-name rule parses what it formats."""
    name = format_file_name("C", "T5-S", 15.0, 750.0, 0, 17, 3)
    assert name == "C_T5S_G15_Z0750_F0_P017_f03.mc"
    fields = parse_file_name(name)
    assert fields == {"procedure": "C", "target_id": "T5-S", "gap_mm": 15.0, "station_z_mm": 750.0, "field": 0,
                      "pose_index": 17, "frame_index": 3}
    assert parse_file_name(format_file_name("A", "T2", None, 500.0, 3, 1, 99))["gap_mm"] is None
    with pytest.raises(ValueError):
        parse_file_name("not_a_capture.mc")


def test_manifest_round_trip(tmp_path: Path):
    """Section 9: every manifest column survives a write and read, poses group by pose key."""
    pose = tilted_pose(10.0, -5.0, TEST_DEPTH_MM, "V", 15.0)
    records = [FrameRecord(tmp_path / format_file_name("A", "T2", None, TEST_DEPTH_MM, 0, 4, f), "A", "T2", None,
                           TEST_DEPTH_MM, 0, 4, f, RigidTransform.from_rotation_vector_degrees([1, 2, 3], [100, 200, 300]),
                           pose, seed=11, offset_h_mm=0.5, offset_v_mm=-0.25, indicator_mm=0.021, timestamp="t",
                           sensor_temp_c=31.5, subseries="tilt", tilt_axis="V", tilt_deg=15.0, metadata={"note": "x"})
               for f in range(3)]
    path = write_manifest_csv(tmp_path / "manifest.csv", records)
    loaded = load_manifest(path)
    assert len(loaded) == 3
    first = loaded[0]
    assert first.pose_key() == records[0].pose_key()
    assert first.target_pose_camera.difference_from(pose) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert first.robot_pose.difference_from(records[0].robot_pose) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert first.indicator_mm == pytest.approx(0.021)
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
    target = targets.get("T5-L", TEST_GAP_MM)
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
    disk = targets.get("T4-L").features_of_kind(FEATURE_DISK)[0]
    cutout = targets.get("T5-L").features_of_kind(FEATURE_CUTOUT)[0]
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
