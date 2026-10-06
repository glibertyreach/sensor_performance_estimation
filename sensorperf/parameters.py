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
"""Section 4: registration captures of the noise plate T2 (plane correspondence)."""
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

# Target identifiers (Section 3.2). One disk plate and one cutout plate; the registration plate T1 no
# longer exists (registration uses the noise plate T2 by plane correspondence, redesign note Section 3).
TARGET_NOISE_PLATE = "T2"
"""Noise plate; also the registration target (plane-only solve)."""
TARGET_RAISED_SQUARE = "T3a"
TARGET_SQUARE_WINDOW = "T3b"
TARGET_DISKS = "T4"
"""The disk plate: feature_count disks, blank sites and a post-only site."""
TARGET_CUTOUTS = "T5"
"""The cutout plate: feature_count cutouts and blank sites."""
TARGET_IDS = (TARGET_NOISE_PLATE, TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW, TARGET_DISKS, TARGET_CUTOUTS)
"""All target identifiers of Section 3.2."""

FIELD_POSITION_CENTER = 0
"""Field position code of the on-axis station."""
FIELD_POSITION_SIGNS = {1: (-1.0, -1.0), 2: (1.0, -1.0), 3: (1.0, 1.0), 4: (-1.0, 1.0)}
"""Field position codes 1-4 and their (H, V) signs; the offset is the sign times
FIELD_OFFSET_FRACTION times the half field at the station depth."""
FIELD_POSITION_CODES = (FIELD_POSITION_CENTER,) + tuple(FIELD_POSITION_SIGNS)
"""All field position codes, center first."""

TIER_A_DISPARITY_QUANTUM_PX = 0.125
"""Default of ``CharacterizationParameters.tier_a_disparity_quantum_px``: the disparity quantum q assumed for the expected
depth quantum dZ_q = q Z^2 / k of the B-Z series until Analysis A (or the ramp) measures one (Section 6.2, Step 3: "using
the Tier-A q"). The value (1/8 px) is the indicative one of the synthetic sensor model and is NOT a datasheet value."""

ROBOT_REPEATABILITY_DEFAULT_MM = 0.05
"""Default of ``CharacterizationParameters.robot_repeatability_mm``: the position repeatability required of the robot
(ISO 9283, Section 3.1 equipment list; the value is the document's requirement, not a measurement)."""
TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO = 2.0
"""The truth rule of Analysis B-Z (Section 11.2, Step 10): a Z-step rung smaller than this many times the robot repeatability
(``robot_repeatability_mm``) is flagged ``truth_reliable = False`` and left out of the gain regression (see
``CharacterizationParameters.truth_reliable_rung_floor_mm``). It is a SEPARATE rule from the floor of the step ladder
(Section 6.2, Step 2), which is ``robot_min_resolvable_move_mm`` alone, an independent equipment figure; the two coincide at
the defaults (2 x 0.05 mm = 0.1 mm), so at the indicative values no rung is flagged, but they are not tied: the rule
follows the repeatability, the ladder floor does not."""

EXPECTED_D0_PX_DEFAULT = 7.0
"""Default of ``CharacterizationParameters.expected_d0_px``: the expected minimum detectable diameter D_0, pixels (about 40
pixels of area; Section 3.2). A module constant so that ``AREA_BIAS_FIT_D0_FACTOR`` can tie the default of the area
edge-bias fit threshold to it."""
AREA_BIAS_FIT_D0_FACTOR = 2.0
"""The edge-bias fit of Analysis C (Section 12, Step 10) uses a feature only where its diameter is at least this many
times the expected minimum detectable diameter ``EXPECTED_D0_PX_DEFAULT``, so that the bias does not depend on the diameter
(``CharacterizationParameters.area_bias_fit_min_d_px``)."""

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

    # Working volume and station ladder (redesign note, Section 1)
    z_min_mm: float = 400.0
    """Near range limit. † Step 4.5 confirms that the sensor reads at this distance."""
    z_max_mm: float = 1600.0
    """Far range limit. † Step 4.5 confirms that the sensor reads at this distance."""
    z_station_ratio: float = 2.0 ** 0.25
    """Ratio of successive stations of the one geometric ladder (four stations per octave); the stations are
    Z_MIN times this ratio to the power k, rounded to 1 mm, up to and including Z_MAX."""
    z_shape_station_stride: int = 2
    """Every this-many-th station is a B-HV shape station (400, 566, 800, 1131, 1600 with the defaults)."""
    z_reduced_station_stride: int = 4
    """Every this-many-th station is a reduced station, used by the slowest tests (B-Z, the A tilt sub-series)
    (400, 800, 1600 with the defaults)."""
    z_reference_mm: float = 800.0
    """The reference station (one of the ladder stations) used for sentinels, the warm-up check, the re-mount check,
    the C field sub-series, the C open-background variant and the D post check."""
    field_offset_fraction: float = 0.6
    """Off-axis field positions at (+/- f W/2, +/- f H/2) from the center, f of the half field (A, C)."""
    tilt_angles_deg: tuple[float, ...] = (0.0, 15.0, 30.0, 45.0)
    """Plate tilts about H and about V in the A tilt sub-series. A tilt is planned only where the plate's near edge
    stays at or beyond Z_MIN (``acquisition.plan.tilt_near_edge_mm``); the others are skipped and listed in
    plan_summary.txt."""

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
    drift_run_duration_min: float = 480.0
    """Duration of the OPTIONAL separate drift run (Section 4, Step 3), minutes: long enough to cover the warm-up and the
    planned session length. The run is made before the session (the same day or the day before) with the robot idle, T2 on a
    fixed stand at the reference station (Z_REFERENCE_MM), fronto-parallel, the sensor powered from cold. Its captures are
    SENTINEL_FRAMES frames each, files in the sentinels folder with the sub-series label ``drift_run``. It costs no robot time
    and is outside the Section 9 budget."""
    drift_run_capture_interval_min: float = 2.0
    """Interval between the captures of the optional drift run, minutes (SENTINEL_FRAMES frames per capture)."""
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
    """Frames per Z-step pose (B-Z ladder visits)."""
    frames_per_ramp_pose: int = 50
    """Frames per pose of the B-Z ramp sub-series (Section 6.2): the ramp is one pose per station, so it takes more
    frames than a ladder visit (the same as a tilt pose of A)."""
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
    tier_a_disparity_quantum_px: float = TIER_A_DISPARITY_QUANTUM_PX
    """Disparity quantum q (pixels) from which the planner derives the expected depth quantum dZ_q(Z0) = q Z0^2 / k of the
    B-Z series (ladder rungs, ramp tilt and the optional staircase step). The default is an indicative value, not a
    datasheet value; once the ramp (or Analysis A) has measured the quantum, override this parameter in the
    ``plan_stations --parameters`` JSON and re-plan."""
    z_step_ladder_quanta: tuple[float, ...] = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0)
    """Commanded rungs of the B-Z step ladder, as multiples of the expected depth quantum at the station,
    dZ_q(Z0) = q Z0^2 / k with the Tier-A q until Analysis A has measured one (Section 6.2). Each rung is raised to at
    least ``robot_min_resolvable_move_mm``. At the indicative geometry the rungs run from 0.1 to 3.1 mm at 400 mm, 0.39 to
    12.4 mm at 800 mm and 1.55 to 50 mm at 1600 mm; the planner lists them per station in plan_summary.txt."""
    robot_min_resolvable_move_mm: float = 0.1
    """The smallest Z move the robot is trusted to execute (Neil's statement of what the robot can resolve, mm; an
    independent parameter, not derived from the repeatability). It is the floor of the step ladder (Section 6.2, Step 2): a
    ladder rung or a staircase step smaller than this is raised to it. The analysis takes the read-back displacement as the
    truth in any case. The separate truth rule of Analysis B-Z (twice ``robot_repeatability_mm``, see
    ``TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO``) gives the same 0.1 mm at the defaults, but the two do not follow each other."""
    robot_repeatability_mm: float = ROBOT_REPEATABILITY_DEFAULT_MM
    """Position repeatability of the robot (ISO 9283), the equipment requirement of Section 3.1. A ladder rung smaller than
    ``TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO`` (2) times this has a step truth too close to the robot's own scatter, so
    Analysis B-Z reports it but flags it ``truth_reliable = False`` (Section 11.2, Step 10)."""
    z_step_approach_overshoot_mm: float = 2.0
    """Every visit of series Z is approached from the same direction so that the backlash and compliance of the robot
    joints (a different elastic and frictional state after a move in the opposite direction) do not enter the A / B
    difference: the robot first moves this far below the pose (to a smaller Z, nearer the sensor) and then moves up onto
    the pose, in the direction of increasing Z."""
    z_step_repeats: int = 10
    """ABAB cycles for each step size."""
    ramp_quanta: float = 4.0
    """The B-Z ramp tilts the plate about H so that the true depth across the plate's VISIBLE height (the smaller of the
    plate height and the field height at that Z) spans this many expected depth quanta (Section 6.2)."""
    ramp_max_intermediate_fraction: float = 0.5
    """Classification of a B-Z ramp curve as stepped or smooth (Section 11.2, Step 14): the curve counts as stepped when
    fewer than this fraction of its changes over a half-quantum window are intermediate (fall inside
    ``ramp_intermediate_band``), and as smooth otherwise. Chosen by argument, not from data: a staircase has about 0 of
    its window changes in the band (a window lies on a plateau or spans one step) and a smooth ramp has all of them, so
    the midpoint 0.5 separates the two with the same margin on each side."""
    ramp_intermediate_band: tuple[float, float] = (0.25, 0.75)
    """Window changes of a B-Z ramp curve, as fractions of the quantum, that count as intermediate (Section 11.2,
    Step 14): a change over a half-quantum window is about 0 (on a plateau) or about 1 quantum (across a step) for a
    staircase and about 0.5 quantum for a smooth ramp, so the band is the central half of the interval between 0 and 1.
    Chosen by argument, not from data."""
    z_staircase_subdivision: int = 10
    """OPTIONAL second pass (staircase): fine-sweep points per expected depth quantum; the step is
    max(dZ_q / this, ``robot_min_resolvable_move_mm``)."""
    z_staircase_quanta: float = 3.0
    """OPTIONAL second pass (staircase): the sweep runs from Z0 to Z0 plus this many expected quanta (Section 6.2). The
    staircase is planned only when ``plan_stations --staircase`` asks for it and is outside the Section 9 budget."""
    z_staircase_frames: int = 10
    """OPTIONAL second pass (staircase): frames per step."""
    lateral_sweep_step_px: float = 0.1
    """OPTIONAL second pass of B-HV (lateral sweep, Section 6.1): lateral step of the edge sweep in pixels at the
    reference station (0.12 mm there). The sweep is planned only when ``plan_stations --lateral-sweep`` asks for it and
    is outside the Section 9 budget."""
    lateral_sweep_span_px: float = 2.0
    """OPTIONAL second pass of B-HV (lateral sweep): span of the sweep in H and in V, pixels at the reference station.
    The sweep runs from one step to the full span from the nominal position, so it has
    ``lateral_sweep_span_px / lateral_sweep_step_px`` poses per axis (20 with the defaults); the nominal position itself
    (offset 0) is the nominal B pose already captured and is not repeated."""
    zstep_patch_sizes_px: tuple[int, ...] = (1, 5, 20)
    """Side lengths of the square patches of the B-Z analysis (1 px, 5 x 5, 20 x 20)."""

    # Detection (D, also used in B-Z)
    detection_false_alarm_target: float = 0.01
    """Target false-alarm rate per window; sets the threshold tau from the blank-site distribution."""
    detection_min_connected_px: int = 2
    """Minimum connected region counted as a detection."""
    detection_window_margin_px: float = 3.0
    """Search window radius equals D/2 in pixels plus this margin."""
    detection_trials_per_level: int = 60
    """Independent trials per (feature, station) pair in the D series."""
    detection_low_trials: int = 300
    """Trials per (feature, station) pair at the detection_low_station_count farthest stations, where the smallest
    feature lies near and below the expected threshold. 300 trials measure a detection probability of 5 percent
    (detection_low_probability) to about +/- 2.5 percent at the confidence level (the extended trials of Section 8)."""
    detection_low_station_count: int = 3
    """Number of farthest stations that carry detection_low_trials trials (1131, 1345 and 1600 mm with the defaults)."""
    detection_low_probability: float = 0.05
    """The lowest detection probability the trials measure: D_5, the size at which the false-alarm-corrected detection
    probability is 5 percent. 300 trials (detection_low_trials) resolve it to about +/- 2.5 percent; a lower level
    (3 percent) would be marginal at that count. The 0 percent point is not measured (see
    detection_zero_prediction_level)."""
    detection_zero_prediction_level: float = 0.01
    """The false-alarm-corrected probability level to which the fitted psychometric curve is extrapolated to give the
    predicted D_0 (Section 13). The result is a PREDICTION from the fitted curve below the lowest measured point
    (D_5), never a measurement: a smooth curve never reaches zero and no finite number of trials proves a
    probability is zero."""
    detection_lapse_rate_max: float = 0.05
    """Upper bound of the lapse rate lambda in the psychometric fit (Section 13, Step 4)."""
    confidence_level: float = 0.95
    """Level of all confidence intervals and bounds (B, C, D, E)."""
    bootstrap_resamples: int = 2000
    """Resamples for bootstrap confidence intervals (C, D, E)."""
    independence_sigma_multiple: float = 2.0
    """Lag-1 autocorrelation of a detection sequence must lie within this many standard
    errors (1/sqrt(n)) of zero (Section 13, Step 1: +/- 2/sqrt(n))."""

    # Boundary bias (E)
    boundary_band_half_width_px: float = 8.0
    """Analysis band on each side of a true edge (E); also the ROI shrink in A, the
    reference-plane exclusion in B and C, and the edge margin of the capture planner's field-of-view fit
    (``acquisition.plan``: a target must lie this many pixels inside the image, so the margin used to pull an
    off-axis pose inward is the same band the analysis shrinks the region of interest by)."""
    area_bias_fit_min_d_px: float = AREA_BIAS_FIT_D0_FACTOR * EXPECTED_D0_PX_DEFAULT
    """Smallest feature diameter, pixels at the station, that enters the fit of the area edge bias b against Z (C, Section 12,
    Step 10): the fit uses the largest feature at the stations where its D_px is at least this, so that b does not depend on
    D. The default is twice (AREA_BIAS_FIT_D0_FACTOR) the 7 px expected minimum detectable diameter (``expected_d0_px``), a
    feature well above the size at which the sensor starts to lose it. The two are tied only through their defaults: if you
    override ``expected_d0_px`` in the parameters JSON, override this one with it."""
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
    """Depths at which the legacy metrics are computed (Section 10, Step 12); series A adds them as extra
    noise stations to the ladder, captured at the center field position only."""
    station_match_tolerance_mm: float = 0.5
    """Two depths closer than this are the same station (the file-name rule rounds a station to 1 mm)."""
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
    feature_ladder_ratio: float = 2.0 * math.sqrt(2.0)
    """Diameter ratio of successive disk and cutout features (a half-octave overlap in subtended pixels between
    neighbors, since one feature covers two octaves of D_px over the Z range)."""
    feature_min_px_at_z_max: float = 3.0
    """Subtended size, in pixels at Z_MAX, of the smallest feature."""
    feature_count: int = 3
    """Features per disk plate and per cutout plate."""
    blank_sites_per_plate: int = 3
    """Blank sites per plate (the guess rate gamma of the detection fit). Blank site i serves feature i and is sized to
    that feature's search window at Z_MAX, D_i + 2 DETECTION_WINDOW_MARGIN_PX p(Z_MAX); at most FEATURE_COUNT."""
    post_sites_per_plate: int = 1
    """Post-only sites per disk plate (the post check: a bare post must not be detected)."""
    feature_isolation_px: float = 30.0
    """Minimum edge-to-edge spacing between features, pixels at Z_MAX (evaluated at the far station so that
    neighbors stay separated there, where a pixel covers the most millimeters). Applies edge to edge between every
    pair of sites of a plate, in both directions (along a row and between rows)."""
    feature_plate_margin_px: float = 8.0
    """Front-plate margin beyond the outermost sites of a disk or cutout plate, pixels at Z_MAX (equal to the boundary
    band BOUNDARY_BAND_HALF_WIDTH_PX, so that the analysis band of an outer cutout edge lies on front material at the
    far station). The plate (sites plus margin) must fit the field of view at Z_MIN with the phase-jitter span and
    the boundary band on every side; make_standard_target_set raises an error when it does not."""
    expected_d0_px: float = EXPECTED_D0_PX_DEFAULT
    """Expected minimum detectable size D_0 in pixels, from the detectability model of Section 3.2 (about 40 pixels of
    area, a diameter of 7 px), used before fabrication to size the disk support posts. Not a measured value: Analysis D
    replaces it with the predicted D_0."""
    post_diameter_fraction_of_d0: float = 0.5
    """Rule for the disk support posts: a post must be thinner than this fraction of the expected D_0 (the post check
    of Section 8, Step 1 confirms that a bare post is not detected). The planned post diameter is
    post_diameter_fraction_of_d0 x expected_d0_px x p(Z_MIN) (:meth:`post_diameter_mm`: about 2 mm at the indicative
    geometry)."""
    frame_check_px: float = 0.5
    """IR-edge to depth-discontinuity agreement required in Step 4.5."""

    # Registration
    registration_poses: int = 30
    """Hand-eye poses spanning the volume (Section 4, Step 6)."""
    registration_tilt_range_deg: float = 20.0
    """Half range of the registration tilts about H and V (+/-)."""
    registration_residual_accept_mm: float = 0.15
    """Acceptance limit of the registration residual, mm RMS."""
    mount_check_depth_mm: float = 800.0
    """Depth of the once-per-mount plane-fit check (Section 4, Step 8)."""
    mount_tilt_tolerance_deg: float = 0.05
    """Largest tilt difference between a mounted target's fitted front plane and the registered pose that the
    mount check of Step 4.8 accepts (dagger: 0.05 degrees moves a plate edge 200 mm from center by about
    0.17 mm). Z is held to registration_residual_accept_mm and H, V to frame_check_px."""

    # Equipment (redesign note, Section 4)
    adapter_remount_repeatability_mm: float = 0.02
    """Repeatability of the target adapter when a target is removed and mounted again (the re-mount check of A)."""
    temperature_log_interval_min: float = 1.0
    """Interval at which the sensor and air temperatures are logged during every capture."""

    # ------------------------------------------------------------------
    # Derived station lists (Section 2 relations and Section 5, Step 1)
    # ------------------------------------------------------------------
    def z_stations_mm(self) -> tuple[float, ...]:
        """The one geometric station ladder of the whole procedure: Z_MIN times Z_STATION_RATIO to the
        power k, rounded to 1 mm, from Z_MIN up to and including Z_MAX (400, 476, 566, 673, 800, 951,
        1131, 1345, 1600 with the defaults; nine stations).

        The last station is Z_MAX itself (the ratio power that reaches it is rounded to the nearest
        integer count, so floating-point error in the ratio cannot drop or duplicate the end)."""
        octaves_span = math.log(self.z_max_mm / self.z_min_mm) / math.log(self.z_station_ratio)
        count = int(round(octaves_span)) + 1
        stations = [float(round(self.z_min_mm * self.z_station_ratio ** k)) for k in range(count)]
        stations[-1] = float(round(self.z_max_mm))
        return tuple(stations)

    def _strided_stations_mm(self, stride: int) -> tuple[float, ...]:
        """Every stride-th station of the ladder, counted from Z_MIN, always including the last (Z_MAX)."""
        stations = self.z_stations_mm()
        chosen = list(stations[::stride])
        if chosen[-1] != stations[-1]:
            chosen.append(stations[-1])
        return tuple(chosen)

    def z_shape_stations_mm(self) -> tuple[float, ...]:
        """The B-HV stations: every Z_SHAPE_STATION_STRIDE-th station including both ends
        (400, 566, 800, 1131, 1600 with the defaults)."""
        return self._strided_stations_mm(self.z_shape_station_stride)

    def z_reduced_stations_mm(self) -> tuple[float, ...]:
        """The B-Z and A tilt sub-series stations: every Z_REDUCED_STATION_STRIDE-th station including
        both ends (400, 800, 1600 with the defaults)."""
        return self._strided_stations_mm(self.z_reduced_station_stride)

    def legacy_extra_stations_mm(self) -> tuple[float, ...]:
        """The LEGACY_METRIC_DEPTHS_MM that are not already ladder stations (700 and 1000 mm with the
        defaults), in ascending order. Series A captures them at the center field position only (the legacy
        metrics are center-box metrics), not at the five field positions of the ladder stations."""
        return tuple(sorted(z for z in self.legacy_metric_depths_mm
                            if all(abs(z - s) > self.station_match_tolerance_mm for s in self.z_stations_mm())))

    def noise_stations_mm(self) -> tuple[float, ...]:
        """The A stations: every ladder station plus the legacy extra stations (700 and 1000 mm), in ascending
        order, so the legacy metrics are computed at the same depths as the existing data. The ladder stations
        are captured at the five field positions, the extra ones at the center only
        (:meth:`legacy_extra_stations_mm`)."""
        return tuple(sorted(self.z_stations_mm() + self.legacy_extra_stations_mm()))

    def detection_low_stations_mm(self) -> tuple[float, ...]:
        """The DETECTION_LOW_STATION_COUNT farthest stations (1131, 1345, 1600 mm), which carry
        DETECTION_LOW_TRIALS trials per feature (the extended trials that measure the low point, D_5)."""
        return self.z_stations_mm()[-self.detection_low_station_count:]

    def post_diameter_mm(self, geometry: SensorGeometry) -> float:
        """The planned disk support post diameter, derived from the rule of Section 3.2: POST_DIAMETER_FRACTION_OF_D0
        times the expected D_0 (EXPECTED_D0_PX) at the nearest station, where a pixel is smallest and the post
        therefore subtends the most pixels, D = fraction x D_0,px x p(Z_MIN) (about 2 mm at the indicative geometry)."""
        return self.post_diameter_fraction_of_d0 * self.expected_d0_px * geometry.pixel_footprint_mm(self.z_min_mm)

    @property
    def truth_reliable_rung_floor_mm(self) -> float:
        """The smallest Z-step rung whose step truth is reliable, mm: ``TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO`` times
        ``robot_repeatability_mm`` (Section 11.2, Step 10). Independent of the ladder floor ``robot_min_resolvable_move_mm``
        (Section 6.2, Step 2), with which it coincides at the defaults (0.1 mm)."""
        return TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO * self.robot_repeatability_mm

    def lateral_sweep_positions_px(self) -> tuple[float, ...]:
        """The positions of the optional lateral sweep along one axis, in pixels from the nominal position:
        k x LATERAL_SWEEP_STEP_PX for k = 1 ... LATERAL_SWEEP_SPAN_PX / LATERAL_SWEEP_STEP_PX (20 values, 0.1 to 2.0 px
        with the defaults). The origin is the nominal B pose, already captured, so it is not repeated."""
        steps = int(round(self.lateral_sweep_span_px / self.lateral_sweep_step_px))
        return tuple(k * self.lateral_sweep_step_px for k in range(1, steps + 1))

    def feature_diameters_mm(self, geometry: SensorGeometry) -> tuple[float, ...]:
        """The disk and cutout diameters D_k = FEATURE_MIN_PX_AT_Z_MAX x p(Z_MAX) x FEATURE_LADDER_RATIO^k,
        k = 0 .. FEATURE_COUNT - 1 (7.0, 19.7 and 55.8 mm at the indicative geometry; 3.0 to 12, 8.5 to 34
        and 24 to 96 px over the Z range)."""
        smallest = self.feature_min_px_at_z_max * geometry.pixel_footprint_mm(self.z_max_mm)
        return tuple(smallest * self.feature_ladder_ratio ** k for k in range(self.feature_count))

    def phase_jitter_span_mm(self, geometry: SensorGeometry, depth_mm: float) -> float:
        """PHASE_JITTER_SPAN_PX converted to millimeters at the station depth."""
        return self.phase_jitter_span_px * geometry.pixel_footprint_mm(depth_mm)

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
