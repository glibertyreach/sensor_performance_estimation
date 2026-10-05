# sensorperf: code design

Status: implemented and integrated, 2026-10-05 (see Section 7 for the status after integration). This document is the
contract every module is written against. The procedure it implements is the
Claude Docs document "VSX3000 Resolution, Area-Fidelity, Detectability, and Noise
Characterization Procedure" (2026-10-04), referred to below by its section
numbers; the technician-facing rendering of that procedure is
`docs/procedures/performance_test_procedure.md`.

## 1. Scope and non-goals

In scope: the acquisition-side tools of Part I (station planning with logged
randomization, the capture manifest, the quick-look check, the robot-to-sensor
registration solve); the analyses of Part II (A noise, B resolution in H, V and
Z, C area fidelity, D detectability, E boundary bias) with their CSV, JSON and
figure outputs and the `forward_model_parameters.json` hand-off to the Tier-A
forward-noise simulator; a synthetic renderer of the same targets that makes
"sample data" so every tool and analysis can be run and tested without the
sensor.

Out of scope: driving the robot or the sensor SDK (the tools read and write
files the capture PC exchanges with them); IR fiducial detection (the
registration solve takes the fiducial solver's per-pose results as input, or
depth-plane fits); any attempt to model the VSX3000's internal matcher (the
synthetic renderer imitates gross features only and labels them indicative).

## 2. Coding rules (binding)

- Python 3.10+, numpy, scipy only (matplotlib for figures, pytest for tests).
  No OpenCV, no scikit-image: marching squares comes from `contourpy` (a
  matplotlib dependency) via `matplotlib.contour` or a small local
  implementation.
- Clarity over terseness. Every module has a docstring saying what it computes,
  for which step of the procedure, and in which conventions. Every function
  with more than a trivial body says what it returns and in which units.
  Comment the reasoning, not only the mechanics: code is not self-commenting.
- No magic numbers. Every arbitrary constant is a field of a dataclass or a
  module-level constant with a docstring saying what it is for, including
  numerical guards. Procedure parameters come from
  `sensorperf.parameters.CharacterizationParameters` and `SensorGeometry`,
  never re-typed.
- Units: millimeters, degrees at public interfaces, pixels for image
  coordinates, seconds and minutes as the procedure says. Angles inside
  numerics may be radians.
- Image arrays are (height, width[, channels]); pixel (u, v) = (column, row).
  Depth is camera z; a no-read is NaN inside the package (the `.mc` zero
  sentinel is converted once, in `io/capture_set.py`).
- U.S. spelling.
- Tests: pytest, deterministic (seeded generators), each test names the step of
  the procedure it checks.

## 3. Module map

```
sensorperf/
  parameters.py          CharacterizationParameters (Section 2 table), SensorGeometry (dagger values), relations   DONE
  io/qt_datastream.py    vendored, verified; do not edit                                                           DONE
  io/matcloud.py         vendored, verified; do not edit                                                           DONE
  io/manifest.py         FrameRecord, file-name rule, CSV read/write, selection and grouping                       DONE
  io/capture_set.py      PoseStack: the frames of one pose with NaN no-reads, camera from the header               DONE
  io/session.py          session folder layout, sensor_config.json, Session loader                                 DONE
  geometry/camera.py     PinholeCamera (vendored)                                                                  DONE
  geometry/transforms.py RigidTransform, fit_rigid_transform (vendored)                                            DONE
  geometry/targets.py    two-plane targets, features, ray casting, visibility, signed distance, standard set      DONE
  geometry/registration.py  Registration record, composition chain, hand-eye and plane-only solves                DONE
  features/depth_features.py  temporal statistics (vendored)                                                       DONE
  features/normals.py    plane-fit normals (vendored)                                                              DONE
  features/planes.py     robust plane fit, plane depth images, normalized height, pixel area                       DONE
  stats/intervals.py     Clopper-Pearson, rule of three, bootstrap helper  DONE
  stats/psychometric.py  psychometric curves, isotonic regression, threshold (floor) model  DONE
  simulate/sensor_model.py  indicative VSX3000-like depth renderer of a TwoPlaneTarget  DONE
  simulate/session.py    write a synthetic session (any subset of series) with its manifest  DONE
  acquisition/plan.py    station and pose lists for registration, A, B-HV, B-Z, C, D, sentinels  DONE
  acquisition/pose_log.py  robot pose log (any rotation convention) -> manifest  DONE
  acquisition/check.py   quick-look check of a capture set, D pilot counts  DONE
  analysis/common.py     ROI masks, reference planes, figure and table writers shared by A-E  DONE
  analysis/noise.py      Analysis A (Section 10)  DONE
  analysis/resolution_lateral.py  Analysis B-HV (Section 11.1)  DONE
  analysis/resolution_depth.py    Analysis B-Z (Section 11.2)  DONE
  analysis/area.py       Analysis C (Section 12)  DONE
  analysis/detection.py  Analysis D (Section 13)  DONE
  analysis/boundary.py   Analysis E (Section 14)  DONE
  analysis/forward_model.py  forward_model_parameters.json assembly  DONE
  cli/plan_stations.py, make_manifest.py, check_captures.py, register.py, simulate.py, analyze.py  DONE
tests/
docs/design/code_design.md (this file)
docs/procedures/performance_test_procedure.md (+ figures/, build/, Review/)
```

## 4. Frames, poses and quantities (binding conventions)

- **Camera frame**: the left IR camera; Z along its optical axis, H = x along
  image columns, V = y along image rows. All ground truth is expressed here.
- **Target frame** (`geometry/targets.py`): origin at the target's reference
  point on the front face (plate center), x = H, y = V, z pointing away from
  the sensor; front face z = 0, back plate z = +G. A fronto-parallel target at
  (H, V, Z) has pose `RigidTransform(identity, [H, V, Z])` (target -> camera).
  `tilted_pose(h, v, z, axis, deg)` rotates it about its H or V axis.
- **Robot**: `robot_pose` in the manifest is the read-back flange pose, flange
  -> base. `Registration.target_to_camera(flange_to_base)` gives the ground
  truth target pose; `Registration.flange_to_base_for(target_to_camera)` the
  pose to command. `registration.json` holds `camera_to_base` and
  `target_to_flange` as row-major 4 x 4 matrices.
- **Z-step truth** (Analysis B-Z): the ground truth of a depth step is the
  read-back robot pose carried into the camera frame through the registration
  (the manifest's `target_pose_camera`). The true displacement between two
  visits is the difference of the registered front-plane depth along the
  optical axis; for the fronto-parallel plate of series Z this is the z
  component of the target pose. The truth is a DIFFERENCE of registered target
  poses between visits, so the registration's translation cancels and its
  rotation error enters only through the cosine of the angle error
  (negligible): only the robot's relative motion accuracy matters. The
  commanded `step_mm` is the label of the rung, never the truth. Rungs whose
  true step is below `robot_repeatability_mm` are flagged
  (`truth_reliable` False): reported, kept in the detection curve, excluded from
  the gain regression; a delta_50 below the smallest reliable rung is reported
  as an upper bound (`delta_50_is_bound`). The robot's own read-back scatter
  (`robot_readback_repeatability_mm`, from the A -> A visit pairs) is reported
  per station. Series Z is approached from below
  (`z_step_approach_overshoot_mm`) and its pose log needs x_mm, y_mm, z_mm with
  at least three decimals (`build_manifest` warns otherwise).
- **Manifest** (`io/manifest.py`): one `FrameRecord` per frame with the Section
  9 columns; `pose_key()` groups frames of a pose, `configuration_key()` groups
  poses of one (procedure, target, gap, station, field, sub-series). Lateral
  phase-jitter offsets are `offset_h_mm`, `offset_v_mm` in the camera frame at
  the station depth; `offsets_px()` converts them with p(Z) = Z / f_x.
  Sub-series labels, tilt, B-Z step and visit columns are defined there.
- **Field positions**: code 0 = center; codes 1-4 = (-,-), (+,-), (+,+), (-,+)
  times FIELD_OFFSET_FRACTION times the half field at the station depth
  (`parameters.FIELD_POSITION_SIGNS`).
- **Surfaces**: `TwoPlaneTarget.intersect_rays` classifies each ray as
  SURFACE_FRONT, SURFACE_BACK or SURFACE_NONE and gives the hit point;
  `visible_from` and `geometric_visibility` give V of Sections 12 and 14 with
  and without the projector condition. `Feature.signed_distance_mm` is
  positive on the front-material side (the s of Sections 11 and 14, once
  divided by p(Z)).
- **Normalized height** h = (Z_back - Z) / (Z_back - Z_front)
  (`features/planes.normalized_height`): 1 on the front surface, 0 on the back.
- **Relations** (Section 2) live on `SensorGeometry`: `pixel_footprint_mm`,
  `diameter_in_pixels`, `subtended_angle_mrad`, `disparity_constant_mm_px`
  (k = f_x B), `depth_quantum_mm`, `disparity_quantum_px`.
- **Dagger values**: `SensorGeometry` fields are None until Step 4.5; code that
  needs them calls `geometry.require(name)` and lets `MissingSensorValue`
  propagate with its message. Simulations and tests use
  `SensorGeometry.indicative()`.

## 5. Interfaces of the modules still to be written

Signatures are binding; bodies are the implementer's. Every function below
takes its thresholds from `CharacterizationParameters` (passed in as `params`)
unless the signature says otherwise.

### stats/intervals.py
```python
def clopper_pearson(successes: int, trials: int, confidence: float) -> tuple[float, float]
    # two-sided exact binomial interval; (0, upper) when successes == 0, (lower, 1) when successes == trials
def clopper_pearson_upper(successes: int, trials: int, confidence: float) -> float
    # one-sided upper bound at the confidence level (Section 13, Step 6); equals 1 - (1-c)^(1/n) for 0 successes
def bootstrap_statistic(groups: Sequence[Any], statistic: Callable[[Sequence[Any]], float | np.ndarray],
                        resamples: int, confidence: float, rng: np.random.Generator) -> BootstrapResult
    # resample GROUPS (poses) with replacement, recompute the statistic; percentile interval
    # BootstrapResult: estimate, lower, upper, samples (array), failures (count of statistic exceptions)
```

### stats/psychometric.py
```python
CURVE_LOGISTIC, CURVE_NORMAL, CURVE_WEIBULL = "logistic", "normal", "weibull"

@dataclass(frozen=True)
class PsychometricFitParameters:
    lapse_rate_max: float            # bound of lambda (params.detection_lapse_rate_max)
    curve: str = CURVE_LOGISTIC
    max_iterations: int ...          # optimizer budget, named

@dataclass
class PsychometricFit:
    curve: str; alpha: float; beta: float; guess_rate: float; lapse_rate: float
    deviance: float; converged: bool; levels: np.ndarray; successes: np.ndarray; trials: np.ndarray
    def probability(self, x) -> np.ndarray             # psi(x) = gamma + (1 - gamma - lambda) F((ln x - alpha)/beta)
    def corrected_probability(self, x) -> np.ndarray   # P*(x) = F(...)
    def threshold(self, p_star: float) -> float        # x at which P* = p_star (D_50 at 0.5, D_10 at 0.1)

def fit_psychometric(levels, successes, trials, guess_rate: float, params: PsychometricFitParameters) -> PsychometricFit
    # x = ln(level); maximum binomial likelihood over alpha, beta > 0, lambda in [0, lapse_rate_max]; gamma fixed
    # F: logistic 1/(1+exp(-z)); normal Phi(z); Weibull in ln x: 1 - exp(-exp(z))  (all with (ln x - alpha)/beta)
    # Section 13, Step 4 and 5: D_50 = exp(alpha) for the logistic and normal; threshold() handles each curve
def fit_all_curves(levels, successes, trials, guess_rate, params) -> dict[str, PsychometricFit]   # best by deviance
def isotonic_threshold(levels, corrected_proportions, p_star: float) -> float
    # pool-adjacent-violators monotone fit over ln(level), then linear interpolation of the crossing (Step 5)
def fit_threshold_model(levels, corrected_proportions, trials, confidence: float) -> ThresholdFit
    # Section 13, Step 7: P* = 0 for D <= D0, 1 - exp(-((D - D0)/s)^k) above; MLE + profile-likelihood interval on D0
    # ThresholdFit: d0, scale, shape, d0_lower, d0_upper, deviance
def corrected_rate(raw_rate, guess_rate) -> float      # (psi - gamma) / (1 - gamma), clipped at 0
```

### simulate/sensor_model.py (INDICATIVE renderer; labeled so in every docstring)
```python
@dataclass(frozen=True)
class SyntheticSensorModel:
    geometry: SensorGeometry                 # intrinsics, baseline, projector, depth LSB
    disparity_noise_px: float                # sigma_d (indicative 0.08 px)
    noise_incidence_exponent: float          # sigma multiplier cos(alpha)^(-m) (indicative 1.3)
    disparity_quantum_px: float              # q; 0 disables quantization (indicative 1/8 px)
    output_lsb_mm: float                     # the depth output LSB applied after the disparity quantizer
    matching_window_px: int                  # odd; box window of the imitation matcher (indicative 7)
    front_preference: float                  # weight of front-surface pixels in the window mean (> 1 = foreground fattening; indicative 2.0)
    min_window_fill: float                   # fraction of the window that must be readable, else no-read (indicative 0.5)
    fixed_pattern_amplitude_mm: float        # per-block static offset amplitude at 1 m, scaled by (Z/1000)^2
    fixed_pattern_block_px: int              # block size of the fixed pattern and of the noise correlation
    fixed_pattern_seed: int
    require_projector: bool                  # whether a read needs projector illumination (Section 12, Step 7 question)
    posts_visible: bool                      # render disk posts as front material
    drift_mm_per_hour: float                 # linear drift of the whole depth field (for sentinels), 0 disables
    @classmethod indicative(cls, geometry: SensorGeometry) -> SyntheticSensorModel

@dataclass
class RenderedFrame:
    depth: np.ndarray            # (H, W) camera z in mm, NaN where no read
    xyz: np.ndarray              # (H, W, 3) float32, zeros where no read (the .mc convention)
    true_surface: np.ndarray     # SURFACE_* per pixel from the ideal ray cast
    true_depth: np.ndarray       # ideal camera z (NaN where SURFACE_NONE)
    visibility: np.ndarray       # geometric V (left, right, projector if required)

def render_frame(model, target: TwoPlaneTarget, pose_camera: RigidTransform, rng, elapsed_hours: float = 0.0) -> RenderedFrame
    # 1 ideal ray cast (targets.intersect_rays from the left camera) -> true depth and surface
    # 2 geometric visibility (geometric_visibility) -> readable mask
    # 3 imitation matcher: window-weighted mean of readable depths with front_preference, no-read below min_window_fill
    # 4 disparity noise: d = k/Z + N(0, sigma_d) per fixed_pattern_block_px block, bilinear back to pixels; incidence multiplier
    # 5 fixed pattern per block (seeded, same every frame); quantize d to disparity_quantum_px; Z = k/d; round to output_lsb_mm
    # 6 drift: add drift_mm_per_hour * elapsed_hours
    # 7 xyz = ray * Z/ray_z, zeros where no read
def write_frame(path, frame: RenderedFrame, geometry: SensorGeometry, extra_header: dict) -> None
    # header: fx, fy, cx, cy, h (width), v (height), cameraName "synthetic", version 2, plus extra_header; matrix "XYZ"
```

### simulate/session.py
```python
def write_synthetic_session(root, params, geometry, model, registration: Registration, targets: TargetSet,
                            plan: list[PlannedCapture], rng, frame_scale: float = 1.0, progress=None) -> Path
    # Renders every PlannedCapture (acquisition/plan.py) into its Section 9 folder and file name, writes
    # manifest.csv, sensor_config.json (geometry filled), registration.json, targets.json, parameters.json,
    # targets_asbuilt.csv (nominal), session_log.md (what was simulated). frame_scale < 1 reduces frames per pose
    # (ceil) for quick tests. The read-back robot pose is the commanded one plus optional repeatability noise
    # (model attribute or argument, named constant, default 0).
```

### acquisition/plan.py
```python
@dataclass
class PlannedCapture:
    procedure: str; target_id: str; gap_mm: float | None; station_z_mm: float; field: int; pose_index: int
    frames: int; subseries: str; seed: int | None; offset_h_mm: float; offset_v_mm: float
    tilt_axis: str; tilt_deg: float; step_mm: float | None; visit: str; level_index: int | None
    target_to_camera: RigidTransform        # the wanted ground-truth pose (offsets and tilt applied)
    order: int                               # acquisition order after randomization
    def file_name(frame) -> str              # io.manifest.format_file_name

def plan_registration(params, geometry, rng) -> list[PlannedCapture]        # Section 4, Step 6
def plan_noise_series(params, geometry, rng, filters_off=False) -> list     # Section 5: main (shuffled), tilt, remount; sentinels interleaved by the budget clock
def plan_edge_series(params, geometry, rng) -> list                          # Section 6.1: T3a, T3b x gaps x stations: nominal + jitter poses
def plan_zstep_series(params, geometry, rng, expected_quantum_mm: Callable[[float], float]) -> list   # Section 6.2: ladder ABAB + staircase
def plan_area_series(params, geometry, rng, open_background=False) -> list   # Section 7: arrays x gaps x stations, field sub-series, open variant
def plan_detection_series(params, geometry, rng, pilot_d50_mm: dict, pilot_d0_mm: dict, extended=True) -> list  # Section 8
def insert_sentinels(plan, params, geometry, seconds_per_frame, move_settle_s) -> list   # a sentinel every DRIFT_SENTINEL_INTERVAL_MIN of estimated clock
def capture_budget(plan, frame_rate_hz, move_settle_s) -> BudgetRow list     # Section 9 table: poses, frames, robot hours per series
def write_plan(path_dir, plan, registration | None) -> poses.csv (+ robot flange poses when a registration is given), plan_summary.txt, plan.png
```
Pose indices are unique within (procedure, target, gap, station, field).
Random offsets are uniform over +/- PHASE_JITTER_SPAN_PX / 2 converted to mm at
the station depth; the seed of every draw is logged in the record. Station
order within a series is shuffled with the logged seed (Section 5, Step 2;
Section 7, Step 1; Section 8, Step 3) in the way each section says.

### acquisition/pose_log.py
```python
# The robot writes one row per captured frame (or per pose, with the frame count): columns
#   file (or pose fields), x_mm, y_mm, z_mm, rotation_type, r1..r9, timestamp, sensor_temp_c, air_temp_c, ambient_ir
# rotation_type as in the calibration repository's make_manifest (none, quaternion_wxyz, quaternion_xyzw,
# euler_zyx_deg, euler_xyz_deg, fixed_xyz_deg, rotvec_deg, matrix); reuse that conversion table verbatim.
def build_manifest(pose_log_csv, captures_dir, plan_csv, registration: Registration, sensor_config_id) -> list[FrameRecord]
    # matches files to plan rows by the Section 9 file name, takes the read-back pose from the log,
    # computes target_pose_camera through the registration, complains by name about unmatched files/rows
```

### acquisition/check.py
```python
@dataclass(frozen=True)
class CheckParameters: min_valid_fraction, border_margin_px, plane_residual_warn_mm, pose_residual_warn_mm, normal_warn_deg
def check_session(session: Session, check_params) -> CheckReport   # per pose: frames, valid fraction, border contact,
    # front-plane fit vs registered front plane (distance and angle), back-plane fit where a back plate exists
def pilot_detection_counts(session, params, station_z_mm, ...) -> dict   # Section 8, Step 1: the Section 13 Step 2 rule on the
    # first frame of each C pose at Z_REFERENCE_MM; per configuration the detection fraction per level, a pilot D_50 and D_0
```

### analysis/* (each writes into session.analysis_dir())
Each analysis exposes `run_<letter>(session: Session, options) -> <Letter>Result`
and `write_outputs(result, out_dir)`; results are dataclasses with the
quantities the procedure names, and the CSV column names are the document's
symbols in ASCII (sigma_t_mm, sigma_fp_mm, sigma_tot_mm, bias_mm, fill_rate,
corr_len_h_px, ...). Figures are written as PNG and SVG. Details per letter are
in the subagent briefs and in the module docstrings; the procedure's step
numbers are cited in the code.

## 6. Synthetic test campaign (tests/)

A session rendered at reduced size (the indicative geometry scaled to 160 x 120
with the same field of view, `SensorGeometry` fields set accordingly) with a few
stations and a few frames per pose is the shared fixture for the acquisition
tools and the analyses; a full-size run of one series each is the integration
check (`python3 -m sensorperf.cli.simulate --quick`). Every test states which
step of the procedure it exercises and its acceptance criterion.

## 7. Status after integration (2026-10-05)

All modules exist and 104 tests pass (`python3 -m pytest -q`, about 90 s on
an idle 4-core container). The synthetic quick session (160 x 120, 115 poses,
376 frames) renders in 3.5 s and the six analyses run on it in 22 s; a
640 x 480 demonstration session (121 poses, 388 frames, 149 MB) renders in
55 s and analyzes in 83 s. On the quick session the analyses recover the
rendering model: sigma_d within 8 percent, the power-law exponent 1.97 against
2, the depth quantum within 1 percent, the correlation length at the analytic
value of the bilinear block interpolation, the rise distance within 5 percent
of the matcher window's analytic value, and the edge offset within 2 percent.

Deviations from the first version of this document that the integration
forced or that the implementers chose, each recorded in the module docstrings:
the D threshold tau is set on the rule's own window statistic so that the
false-alarm rate of the rule (after the connected-pixel criterion) equals the
target, instead of a per-pixel quantile; the D bootstrap refits only the best
curve family and the model D_0 interval is the profile-likelihood one; the
staircase quantum falls back to the pooled depth levels (the phase-resultant
estimator of the 6DOF repository's repeatability module) when the single-pixel
noise leaves no resolved plateaus; the plan's field-of-view fit pulls a pose
inward to the largest offset that fits (a plate may cover the whole image
across an axis); the D shuffle is applied within each target mounting; the
legacy repeatability metrics follow testZRepeatabilityBrownBoard.py's
definitions (ddof 0, boxes skipped when outside the image).

Design change (2026-10-05): the dial indicator on the target adapter, the first version's step truth of series Z, is removed; the read-back robot pose through the registration is the step truth (Section 4).
