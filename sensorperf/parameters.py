"""
Every arbitrary constant of the characterization procedure, as a named
parameter (procedure document, Section 2), plus the five geometric relations
of that section and the derived station lists.

The attribute names are the parameter names of the procedure document in lower
case, so a value quoted there can be found here by name and the other way
round. The suggested values of the document are the defaults. A value the
document marks with a dagger (†) depends on VSX3000 datasheet or SDK data that
was not available when the procedure was written; it is ``None`` here until
Step 4.5 of the procedure fills it in, and any computation that needs it raises
a clear error instead of using a guess. The ``SensorGeometry.indicative()``
constructor carries the indicative values of the earlier calibration work
(640 x 480, f about 688 px) for simulations and tests, labeled as such.

Units: millimeters, degrees, pixels, seconds and minutes, as the document says
per parameter. Nothing here is a magic number: a value that is used in a
formula is a field of one of the dataclasses below.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Procedure identifiers (the letters used in file names and the manifest)
# ---------------------------------------------------------------------------
PROCEDURE_REGISTRATION = "R"
"""Section 4: registration captures of target T1."""
PROCEDURE_NOISE = "A"
"""Section 5: noise-plate series."""
PROCEDURE_EDGES = "B"
"""Section 6.1: edge series (lateral resolution and boundary bias)."""
PROCEDURE_ZSTEP = "Z"
"""Section 6.2: Z-step series (depth resolution). The document calls it B-Z;
a separate letter keeps the file names unambiguous."""
PROCEDURE_AREA = "C"
"""Section 7: disk and cutout area series."""
PROCEDURE_DETECTION = "D"
"""Section 8: detection trials."""
PROCEDURE_SENTINEL = "S"
"""Drift sentinels captured during any series (Section 5, Step 3)."""
PROCEDURES = (PROCEDURE_REGISTRATION, PROCEDURE_NOISE, PROCEDURE_EDGES, PROCEDURE_ZSTEP,
              PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_SENTINEL)
"""All procedure letters accepted in a manifest."""

# Target identifiers (Section 3.2). File names drop the hyphen (T4-S -> T4S).
TARGET_REGISTRATION_PLATE = "T1"
TARGET_NOISE_PLATE = "T2"
TARGET_RAISED_SQUARE = "T3a"
TARGET_SQUARE_WINDOW = "T3b"
TARGET_DISKS_SMALL = "T4-S"
TARGET_DISKS_LARGE = "T4-L"
TARGET_CUTOUTS_SMALL = "T5-S"
TARGET_CUTOUTS_LARGE = "T5-L"
TARGET_IDS = (TARGET_REGISTRATION_PLATE, TARGET_NOISE_PLATE, TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW,
              TARGET_DISKS_SMALL, TARGET_DISKS_LARGE, TARGET_CUTOUTS_SMALL, TARGET_CUTOUTS_LARGE)
"""All target identifiers of Section 3.2."""

FIELD_POSITION_CENTER = 0
"""Field position code of the on-axis station."""
FIELD_POSITION_SIGNS = {1: (-1.0, -1.0), 2: (1.0, -1.0), 3: (1.0, 1.0), 4: (-1.0, 1.0)}
"""Field position codes 1-4 and their (H, V) signs; the offset is the sign times
FIELD_OFFSET_FRACTION times the half field at the station depth."""
FIELD_POSITION_CODES = (FIELD_POSITION_CENTER,) + tuple(FIELD_POSITION_SIGNS)
"""All field position codes, center first."""

# Numerical guards.
MIN_POSITIVE_DEPTH_MM = 1.0e-6
"""A depth smaller than this is treated as zero in the relations below."""


class MissingSensorValue(ValueError):
    """A dagger (†) sensor value is needed but has not been filled in (Step 4.5)."""


# ---------------------------------------------------------------------------
# Sensor geometry (the † values of Section 2)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SensorGeometry:
    """Left IR camera intrinsics, stereo baseline, projector offset and depth
    LSB of the VSX3000. ``None`` means not yet known (Section 2, †)."""

    sensor_fx_px: float | None = None
    """Left IR focal length along image columns (H), pixels. †"""
    sensor_fy_px: float | None = None
    """Left IR focal length along image rows (V), pixels. †"""
    sensor_cx_px: float | None = None
    """Left IR principal point column, pixels. †"""
    sensor_cy_px: float | None = None
    """Left IR principal point row, pixels. †"""
    image_width_px: int | None = None
    """Depth image width in pixels."""
    image_height_px: int | None = None
    """Depth image height in pixels."""
    sensor_baseline_mm: float | None = None
    """Stereo baseline B. The right camera center lies at (+B, 0, 0) in the left
    camera frame (H along the baseline; Step 4.5 confirms the direction). †"""
    projector_offset_mm: tuple[float, float, float] | None = None
    """Projector center relative to the left camera, (H, V, Z) in mm. †"""
    depth_lsb_mm: float | None = None
    """Least significant bit of the depth output, mm. †"""
    frame_rate_hz: float | None = None
    """Frame rate in the chosen trigger mode (capture budget only). †"""

    @classmethod
    def indicative(cls) -> "SensorGeometry":
        """INDICATIVE values for simulation and tests only, for the first VSX3000 unit
        (VSm01, 640 x 480): the header intrinsics recorded in the 6DOF-via-2D-and-3D
        repository (sixdof/core/camera.py, VSX3000_HEADER_*), the 75 mm stereo baseline
        and the projector 40 mm beside the lens noted there (sixdof/render/raycast_renderer.py,
        VSX3000_STEREO_BASELINE_MM; sixdof/core/types.py, PointLight), a 0.1 mm depth LSB
        and 10 frames per second (assumptions). None of these is a confirmed datasheet value."""
        return cls(sensor_fx_px=688.1552734375, sensor_fy_px=688.083251953125,
                   sensor_cx_px=292.3155517578125, sensor_cy_px=256.7590026855469,
                   image_width_px=640, image_height_px=480,
                   sensor_baseline_mm=75.0, projector_offset_mm=(40.0, 0.0, 0.0),
                   depth_lsb_mm=0.1, frame_rate_hz=10.0)

    @classmethod
    def indicative_vsm04(cls) -> "SensorGeometry":
        """INDICATIVE values for the second VSX3000 unit (VSm04, 1280 x 960): the header
        intrinsics recorded in the 6DOF-via-2D-and-3D repository (sixdof/core/camera.py,
        VSX3000_VSM04_HEADER_*); baseline, projector offset, depth LSB and frame rate as in
        indicative(). Not confirmed datasheet values."""
        return cls(sensor_fx_px=1043.956, sensor_fy_px=1043.956, sensor_cx_px=639.494, sensor_cy_px=477.306,
                   image_width_px=1280, image_height_px=960,
                   sensor_baseline_mm=75.0, projector_offset_mm=(40.0, 0.0, 0.0),
                   depth_lsb_mm=0.1, frame_rate_hz=10.0)

    def require(self, name: str) -> Any:
        """The named value, or MissingSensorValue if it is still None."""
        value = getattr(self, name)
        if value is None:
            raise MissingSensorValue(
                f"SensorGeometry.{name} is not known (a † value of Section 2); fill it in from the SDK or "
                "datasheet in Step 4.5 of the procedure, or use SensorGeometry.indicative() for a simulation")
        return value

    def disparity_constant_mm_px(self) -> float:
        """k = f_x B in mm·px, so that disparity d = k / Z (Section 2)."""
        return float(self.require("sensor_fx_px")) * float(self.require("sensor_baseline_mm"))

    def pixel_footprint_mm(self, depth_mm: float) -> float:
        """p(Z) = Z / f_x: the lateral size of one pixel at depth Z, mm."""
        return float(depth_mm) / float(self.require("sensor_fx_px"))

    def diameter_in_pixels(self, diameter_mm: float, depth_mm: float) -> float:
        """D_px = D f_x / Z."""
        return float(diameter_mm) * float(self.require("sensor_fx_px")) / max(float(depth_mm), MIN_POSITIVE_DEPTH_MM)

    def subtended_angle_mrad(self, diameter_mm: float, depth_mm: float) -> float:
        """theta = D / Z in milliradians (small-angle form, as the document uses)."""
        return 1000.0 * float(diameter_mm) / max(float(depth_mm), MIN_POSITIVE_DEPTH_MM)

    def depth_quantum_mm(self, disparity_quantum_px: float, depth_mm: float) -> float:
        """delta Z_q = q Z^2 / k: the depth step implied by a disparity quantum q at depth Z."""
        return float(disparity_quantum_px) * float(depth_mm) ** 2 / self.disparity_constant_mm_px()

    def disparity_quantum_px(self, depth_quantum_mm: float, depth_mm: float) -> float:
        """q = delta Z_q k / Z^2, the inverse of depth_quantum_mm."""
        return float(depth_quantum_mm) * self.disparity_constant_mm_px() / max(float(depth_mm), MIN_POSITIVE_DEPTH_MM) ** 2

    def half_field_mm(self, depth_mm: float) -> tuple[float, float]:
        """Half width and half height of the image footprint at depth Z (mm, along H and V)."""
        return (float(depth_mm) * float(self.require("image_width_px")) / 2.0 / float(self.require("sensor_fx_px")),
                float(depth_mm) * float(self.require("image_height_px")) / 2.0 / float(self.require("sensor_fy_px")))


# ---------------------------------------------------------------------------
# The procedure parameters (Section 2 table, in table order)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CharacterizationParameters:
    """The parameter table of Section 2. Field names are the document's names
    in lower case; docstrings are the document's meanings."""

    # Working volume
    z_min_mm: float = 500.0
    """Near range limit."""
    z_max_mm: float = 1000.0
    """Far range limit."""
    z_noise_step_mm: float = 50.0
    """Spacing of the noise stations (A); 500 to 1000 in steps of 50 gives 11 stations."""
    z_shape_stations_mm: tuple[float, ...] = (500.0, 625.0, 750.0, 875.0, 1000.0)
    """Stations for the edge, area and detection tests (B, C, D)."""
    z_reduced_stations_mm: tuple[float, ...] = (500.0, 750.0, 1000.0)
    """Subset of stations for the slowest tests (B-Z, the D 0 percent series, the A tilt sub-series)."""
    z_reference_mm: float = 750.0
    """The mid-range station used for sentinels, the warm-up check, the re-mount check,
    the field sub-series and the D pilot (the document writes 750 mm in each place)."""
    field_offset_fraction: float = 0.6
    """Off-axis field positions at (+/- f W/2, +/- f H/2) from the center, f of the half field (A, C)."""
    tilt_angles_deg: tuple[float, ...] = (0.0, 15.0, 30.0, 45.0)
    """Plate tilts about H and about V in the A tilt sub-series."""

    # Capture control
    warmup_min_minutes: float = 45.0
    """Minimum powered time before any capture."""
    warmup_drift_window_min: float = 10.0
    """Window over which warm-up drift is judged."""
    warmup_drift_fraction_of_sigma: float = 0.1
    """Allowed drift of the mean plane Z over the window, as a fraction of sigma_t at that Z."""
    warmup_check_interval_min: float = 1.0
    """Interval of the warm-up captures (Step 4.3: 10 frames every minute)."""
    warmup_check_frames: int = 10
    """Frames per warm-up capture (Step 4.3)."""
    drift_sentinel_interval_min: float = 60.0
    """Interval between drift sentinel captures."""
    sentinel_frames: int = 30
    """Frames per drift sentinel (Section 5, Step 3)."""
    robot_settle_time_s: float = 2.0
    """Wait after motion before a capture; Step 4.4 verifies it."""
    settle_check_frames: int = 100
    """Frames of each settle-and-vibration capture (Step 4.4)."""
    settle_sigma_excess_fraction: float = 0.10
    """Servo-on sigma_t may exceed brakes-on sigma_t by at most this fraction (Step 4.4)."""
    frames_per_noise_station: int = 100
    """Frames per Z, field and tilt pose in A (the tilt sub-series uses frames_per_tilt_pose)."""
    frames_per_tilt_pose: int = 50
    """Frames per pose of the A tilt sub-series (Section 5, Step 5)."""
    frames_per_edge_pose: int = 30
    """Frames per edge pose (B-HV)."""
    frames_per_zstep_pose: int = 10
    """Frames per Z-step pose (B-Z)."""
    frames_per_area_pose: int = 10
    """Frames per area pose (C)."""
    frames_per_detection_trial: int = 1
    """Frames per detection trial (D): one, giving single-frame detectability."""
    frames_per_registration_pose: int = 10
    """Frames per registration pose (Section 4; the document's REGISTRATION_FRAMES)."""
    move_and_settle_time_s: float = 3.0
    """Robot move plus settle time per pose assumed by the capture budget (Section 9)."""

    # Phase randomization
    phase_jitter_span_px: float = 8.0
    """Square span of the random lateral offsets in pixels at the station depth (B, C, D)."""
    phase_jitter_poses_edge: int = 25
    """Random-offset poses per edge station (B-HV)."""
    phase_jitter_poses_area: int = 30
    """Random-offset poses per area configuration (C)."""
    field_subseries_poses_area: int = 10
    """Random-offset poses per array at each off-axis field position (Section 7, Step 3)."""

    # Z-step test
    z_step_ladder_mm: tuple[float, ...] = (0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 4.0)
    """Commanded step sizes of the B-Z ladder."""
    robot_repeatability_mm: float = 0.05
    """Position repeatability of the robot (ISO 9283), the equipment requirement of Section 3.1; a commanded Z step
    smaller than this has a step truth no better than the robot itself, so such ladder rungs are reported but flagged."""
    z_step_approach_overshoot_mm: float = 2.0
    """Every visit of series Z is approached from the same direction so that the backlash and compliance of the robot
    joints (a different elastic and frictional state after a move in the opposite direction) do not enter the A / B
    difference: the robot first moves this far below the pose (to a smaller Z, nearer the sensor) and then moves up onto
    the pose, in the direction of increasing Z."""
    z_step_repeats: int = 10
    """ABAB cycles for each step size."""
    z_staircase_subdivision: int = 10
    """Fine-sweep points per expected depth quantum."""
    z_staircase_quanta: float = 3.0
    """The fine staircase sweeps from Z0 to Z0 plus this many expected quanta (Section 6.2, Step 3)."""
    z_staircase_frames: int = 10
    """Frames per staircase step."""
    zstep_patch_sizes_px: tuple[int, ...] = (1, 5, 20)
    """Side lengths of the square patches of the B-Z analysis (1 px, 5 x 5, 20 x 20)."""

    # Detection (D, also used in B-Z)
    detection_false_alarm_target: float = 0.01
    """Target false-alarm rate per window; sets the threshold tau from the blank-site distribution."""
    detection_min_connected_px: int = 2
    """Minimum connected region counted as a detection."""
    detection_window_margin_px: float = 3.0
    """Search window radius equals D/2 in pixels plus this margin."""
    detection_levels: int = 9
    """Diameter levels per psychometric curve."""
    detection_level_low_factor: float = 0.3
    """Smallest level as a multiple of the pilot D_50."""
    detection_level_high_factor: float = 2.5
    """Largest level as a multiple of the pilot D_50."""
    detection_trials_per_level: int = 60
    """Independent trials per level in the main D series."""
    detection_zero_trials: int = 300
    """Trials per level in the extended 0 percent series."""
    detection_zero_probability_bound: float = 0.01
    """Upper bound on detection probability that defines practically zero detection."""
    detection_lapse_rate_max: float = 0.05
    """Upper bound of the lapse rate lambda in the psychometric fit (Section 13, Step 4)."""
    detection_fine_ladder_ratio: float = 2.0 ** 0.25
    """Diameter ratio of a dedicated fine detection plate (Section 8, Step 2)."""
    confidence_level: float = 0.95
    """Level of all confidence intervals and bounds (B, C, D, E)."""
    bootstrap_resamples: int = 2000
    """Resamples for bootstrap confidence intervals (C, D, E)."""
    independence_sigma_multiple: float = 2.0
    """Lag-1 autocorrelation of a detection sequence must lie within this many standard
    errors (1/sqrt(n)) of zero (Section 13, Step 1: +/- 2/sqrt(n))."""

    # Boundary bias (E)
    boundary_band_half_width_px: float = 8.0
    """Analysis band on each side of a true edge (E); also the ROI shrink in A and the
    reference-plane exclusion in B and C."""
    boundary_bin_width_px: float = 0.25
    """Signed-distance bin width (B, E): four bins per pixel."""
    surface_assignment_sigma_multiple: float = 3.0
    """A read within this many sigma_tot(Z) of a reference plane is assigned to that plane (E)."""
    front_read_height_threshold: float = 0.5
    """Normalized height h above which a read counts as a front read (C, Section 12, Step 3)."""
    esf_rise_low: float = 0.1
    """Lower normalized height of the rise distance (10 percent)."""
    esf_rise_high: float = 0.9
    """Upper normalized height of the rise distance (90 percent)."""
    esf_half_height: float = 0.5
    """Normalized height of the edge crossing s_50 (B-HV, Step 8) and of the area iso-contour (C, Step 5)."""
    esf_linearity_tolerance_h: float = 0.1
    """Largest allowed difference in h between the small-gap and large-gap ESFs before the
    result is marked step-height dependent (Section 11.1, Step 7)."""
    esf_agreement_bins: float = 1.0
    """The robot-stepped and slanted-edge ESFs must agree within this many bins once aligned (Step 5)."""
    noise_closure_tolerance: float = 0.2
    """Allowed relative mismatch of sigma_tot^2 against sigma_t^2 + sigma_fp^2 + bias^2 (Section 10, Step 5)."""
    quantization_patch_px: int = 20
    """Side of the central patch whose depth codes are histogrammed (Section 10, Step 8)."""
    legacy_box_half_px: int = 10
    """Half side of the legacy 20 x 20 px boxes of testZRepeatabilityBrownBoard.py (Step 12)."""
    legacy_box_centers_px: tuple[tuple[int, int], ...] = ((264, 253), (137, 81), (401, 83), (404, 386), (129, 392))
    """The five VSX3000 BrownBoard box centers (column, row) of testZRepeatabilityBrownBoard.py."""
    legacy_metric_depths_mm: tuple[float, ...] = (700.0, 1000.0)
    """Depths at which the legacy metrics are computed (Section 10, Step 12)."""
    autocorrelation_threshold: float = 1.0 / math.e
    """The correlation length is the lag where the normalized autocorrelation first falls to this (1/e)."""

    # Targets
    noise_plate_size_mm: tuple[float, float] = (400.0, 400.0)
    """Uniform matte plate (T2), width x height."""
    plate_flatness_mm: float = 0.05
    """Required flatness of every plate."""
    plate_flatness_sigma_fraction: float = 0.25
    """The flatness must be at most this fraction of the smallest expected sigma_tot."""
    gap_small_mm: float = 15.0
    """Small front-to-back plate distance G."""
    gap_large_mm: float = 60.0
    """Large front-to-back plate distance G."""
    edge_slant_deg: float = 5.0
    """Rotation of the edge target in its own plane relative to the image axes (B-HV)."""
    edge_square_size_mm: float = 160.0
    """Side of the raised square (T3a) and of the square window (T3b)."""
    chamfer_margin_deg: float = 10.0
    """Margin added to the worst-case ray angle in the bevel check (Section 3.3)."""
    edge_land_max_mm: float = 0.2
    """Maximum residual flat land at a knife edge."""
    diameter_ladder_ratio: float = math.sqrt(2.0)
    """Ratio between successive diameters of the disk and cutout ladder."""
    diameter_min_footprint_fraction: float = 0.3
    """Smallest diameter as a fraction of the pixel footprint at Z_MIN."""
    diameter_max_footprint_multiple: float = 30.0
    """Largest diameter as a multiple of the pixel footprint at Z_MAX."""
    feature_isolation_px: float = 30.0
    """Minimum edge-to-edge spacing between features, pixels at Z_MIN."""
    post_diameter_fraction_of_d0: float = 0.5
    """Disk support posts must be thinner than this fraction of the pilot D_0."""
    frame_check_px: float = 0.5
    """IR-edge to depth-discontinuity agreement required in Step 4.5."""

    # Registration
    registration_poses: int = 30
    """Hand-eye poses spanning the volume (Section 4, Step 6)."""
    registration_tilt_range_deg: float = 20.0
    """Half range of the registration tilts about H and V (+/-)."""
    registration_residual_accept_mm: float = 0.15
    """Acceptance limit of the registration residual, mm RMS."""
    mount_check_depth_mm: float = 750.0
    """Depth of the once-per-mount plane-fit check (Section 4, Step 8)."""

    # Open-background variant and continuous-angle variant (optional series)
    continuous_angle_step_mm: float = 10.0
    """Z step of the optional continuous-angle D variant (Section 8, Step 7)."""
    continuous_angle_poses_per_step: int = 20
    """Random-offset poses per Z step of that variant."""

    # ------------------------------------------------------------------
    # Derived station lists (Section 2 relations and Section 5, Step 1)
    # ------------------------------------------------------------------
    def noise_stations_mm(self) -> tuple[float, ...]:
        """Z_MIN : Z_NOISE_STEP_MM : Z_MAX inclusive (11 stations with the defaults)."""
        count = int(round((self.z_max_mm - self.z_min_mm) / self.z_noise_step_mm)) + 1
        return tuple(self.z_min_mm + index * self.z_noise_step_mm for index in range(count))

    def diameter_ladder_mm(self, geometry: SensorGeometry) -> tuple[float, ...]:
        """The disk and cutout diameters: from DIAMETER_MIN_FOOTPRINT_FRACTION x p(Z_MIN) up to
        DIAMETER_MAX_FOOTPRINT_MULTIPLE x p(Z_MAX) in steps of DIAMETER_LADDER_RATIO
        (Section 3.2). The last rung is the first one at or above the maximum."""
        smallest = self.diameter_min_footprint_fraction * geometry.pixel_footprint_mm(self.z_min_mm)
        largest = self.diameter_max_footprint_multiple * geometry.pixel_footprint_mm(self.z_max_mm)
        rungs = [smallest]
        while rungs[-1] < largest:
            rungs.append(rungs[-1] * self.diameter_ladder_ratio)
        return tuple(rungs)

    def phase_jitter_span_mm(self, geometry: SensorGeometry, depth_mm: float) -> float:
        """PHASE_JITTER_SPAN_PX converted to millimeters at the station depth."""
        return self.phase_jitter_span_px * geometry.pixel_footprint_mm(depth_mm)

    def detection_level_diameters_mm(self, pilot_d50_mm: float) -> tuple[float, ...]:
        """DETECTION_LEVELS diameters, log-spaced from DETECTION_LEVEL_LOW_FACTOR to
        DETECTION_LEVEL_HIGH_FACTOR times the pilot D_50 (Section 8, Step 2)."""
        low = math.log(self.detection_level_low_factor * pilot_d50_mm)
        high = math.log(self.detection_level_high_factor * pilot_d50_mm)
        if self.detection_levels == 1:
            return (math.exp((low + high) / 2.0),)
        return tuple(math.exp(low + (high - low) * index / (self.detection_levels - 1))
                     for index in range(self.detection_levels))

    def rule_of_three_bound(self, trials: int) -> float:
        """One-sided upper bound on a probability after zero successes in n trials at the
        confidence level: 1 - (1 - c)^(1/n), about 3/n at 95 percent (Section 8, Step 4)."""
        if trials < 1:
            raise ValueError("trials must be at least 1")
        return 1.0 - (1.0 - self.confidence_level) ** (1.0 / trials)

    # ------------------------------------------------------------------
    # Serialization (the session's parameter record)
    # ------------------------------------------------------------------
    def to_json(self, path: str | Path) -> Path:
        """Write the parameters as JSON (lists for tuples)."""
        Path(path).write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")
        return Path(path)

    @classmethod
    def from_json(cls, path: str | Path) -> "CharacterizationParameters":
        """Read parameters written by to_json; unknown keys are an error, tuples are restored."""
        document = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(document)

    @classmethod
    def from_dict(cls, document: dict) -> "CharacterizationParameters":
        """Build from a dict of overrides; keys must be field names."""
        known = {f.name: f for f in fields(cls)}
        unknown = [key for key in document if key not in known]
        if unknown:
            raise ValueError(f"unknown parameter names {unknown}; see sensorperf.parameters.CharacterizationParameters")
        values: dict[str, Any] = {}
        for key, value in document.items():
            default = getattr(cls(), key)
            if isinstance(default, tuple):
                value = tuple(tuple(v) if isinstance(v, list) else v for v in value)
            values[key] = value
        return cls(**values)

    def document_names(self) -> dict[str, str]:
        """Field name -> the upper-case parameter name used in the procedure document."""
        return {f.name: f.name.upper() for f in fields(self)}


def parameter_table_rows(params: CharacterizationParameters) -> list[tuple[str, str, str]]:
    """(document name, value, meaning) for every parameter, for the generated
    parameter table of the procedure document. The meaning is the field's
    docstring, read from the class source."""
    import inspect
    source = inspect.getsource(CharacterizationParameters)
    rows: list[tuple[str, str, str]] = []
    for f in fields(params):
        value = getattr(params, f.name)
        if isinstance(value, tuple):
            text = ", ".join(_format_value(v) for v in value)
        else:
            text = _format_value(value)
        rows.append((f.name.upper(), text, _docstring_after(source, f.name)))
    return rows


def _format_value(value: Any) -> str:
    if isinstance(value, tuple):
        return "(" + ", ".join(_format_value(v) for v in value) + ")"
    if isinstance(value, float):
        if value == int(value) and abs(value) < 1.0e6:
            return str(int(value))
        return f"{value:.4g}"
    return str(value)


def _docstring_after(source: str, name: str) -> str:
    """The triple-quoted docstring that follows the field ``name`` in the class source."""
    marker = f"\n    {name}:"
    start = source.find(marker)
    if start < 0:
        return ""
    quote = source.find('"""', start)
    end = source.find('"""', quote + 3)
    if quote < 0 or end < 0:
        return ""
    return " ".join(source[quote + 3:end].split())
