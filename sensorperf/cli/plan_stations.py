"""
Command line: plan the stations and poses of the characterization capture
(procedure document, Sections 4 to 9).

    python3 -m sensorperf.cli.plan_stations --out plan/ --seed 1
    python3 -m sensorperf.cli.plan_stations --out plan/ --registration registration.json \\
        --sensor-config sensor_config.json --series A C --seed 7

What it computes
----------------
Every commanded pose of the series of Part I, in the order of the procedure:
the registration poses of Section 4 (R, on the noise plate T2, from Z_MIN to Z_MAX), the noise series of Section 5 (A:
main stations in a seeded random order, tilt sub-series, repeat-mount check), the edge series
of Section 6.1 (B), the Z-step series of Section 6.2 (Z: step ladder with rungs in expected quanta at the reduced
stations, and one tilted ramp pose at every station; ``--staircase`` adds the optional fine staircase), the area series of
Section 7 (C: jittered poses, field sub-series, optional open-background
variant) and the detection series of Section 8 (D: main trials at every station and
extended trials of the low point, D_5, at the farthest stations),
with drift sentinels inserted on the budget clock. ``--filters-off`` appends the filters-off repeat of A, B-HV and
B-Z (Section 4, Step 4.2) after each filters-on series; the summary lists it outside the main budget. The drift sentinels
captured during an optional set (filters-off repeat, staircase, lateral sweep) belong to that set: they carry its sub-series label
and pose-index range and are counted in its own table, never in the main budget. Every pose records the direction it is
approached from (Part I: from below in Z and from -H and -V; ``approach_direction`` in the notes and the manifest). Two further
options, both off by default: ``--lateral-sweep`` adds the optional second pass of B-HV (Section 6.1, Step 6: T3a swept
in H and then in V in steps of LATERAL_SWEEP_STEP_PX over LATERAL_SWEEP_SPAN_PX at Z_REFERENCE_MM, outside the main
budget), ``--drift-run`` adds the optional separate drift run (Section 4, Step 3: T2 on a fixed stand at Z_REFERENCE_MM, the
robot idle, one capture every DRIFT_RUN_CAPTURE_INTERVAL_MIN minutes for DRIFT_RUN_DURATION_MIN minutes, 241 captures with the
default parameters, outside the main budget and with no drift sentinels around it), and ``--reuse-c-first-frames`` lets the first frame of each C pose count as a D trial (Section 8, Reuse), so the
D main series plans only the remaining poses (the Section 9 budget is still computed without the reuse). Every random draw uses
``np.random.default_rng(seed)`` with a seed derived from ``--seed`` and logged in
poses.csv. A target that would not fit the field of view at its station is pulled
inward along its field direction and the summary says so (Section 5, Step 1).

Drift run on a fixed stand (``--drift-run``)
    The plate stands still on a fixed stand and the robot is idle, so these captures have no read-back robot pose. Their
    rows in poses.csv carry the NOMINAL pose of T2 at the reference station (center field, fronto-parallel) and the notes
    field ``fixed_stand=true`` (the JSON notes read ``"fixed_stand": true``). ``make_manifest`` copies the robot pose columns
    and the target pose of the manifest from that nominal pose, so the pose log of the run needs only the capture time
    (``timestamp``) and the sensor temperature (``sensor_temp_c``) per capture and leaves the robot pose columns empty; the
    registered pose is constant for the run, and the drift analysis uses only the relative mean Z against the first capture
    after the settling, so an absolute error of the stand's pose does not enter. The run is taken on its own day: to plan it
    alone give ``--series`` without letters (``--drift-run --series``) and use the same registration as for the session.

Frames and conventions (millimeters, degrees at the interface)
    camera frame  the left IR camera: Z along the optical axis, H = x along
                  image columns, V = y along image rows.
    target pose   target -> camera (poses.csv columns target_x_mm ... target_rz_deg:
                  x, y, z and a rotation vector in degrees).
    flange pose   with --registration: the flange pose to command in the robot
                  base frame (base_x_mm ... base_rz_deg, r00..r22, quat_*).

Inputs
    --registration PATH   registration.json (optional). Without it, poses.csv holds
                          the target poses in the camera frame only.
    --sensor-config PATH  sensor_config.json whose geometry gives the intrinsics,
                          baseline and frame rate (optional). Without it the
                          INDICATIVE geometry of the earlier calibration work is used
                          (640 x 480, f about 688 px, 10 frames/s); a loud note is
                          printed and written to the summary, because the Section 9
                          budget and the field fit depend on it.
    --parameters PATH     parameters.json with overrides of the Section 2 table (any parameter, for example
                          tier_a_disparity_quantum_px, the measured quantum behind the B-Z step sizes).

Outputs (in --out)
    poses.csv, plan_summary.txt, plan.png, targets.json, parameters.json

Exit code: 0 when the plan was written, 2 when an input must be fixed (the
message says what to change).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from sensorperf.acquisition.plan import (
    PLAN_SUMMARY_NAME, PLANNED_SERIES, PlanDiagnostics, plan_full_session, write_plan,
)
from sensorperf.geometry.registration import Registration
from sensorperf.geometry.targets import TARGETS_FILE_NAME, make_standard_target_set
from sensorperf.io.session import PARAMETERS_FILE_NAME, SensorConfig
from sensorperf.parameters import CharacterizationParameters, MissingSensorValue, SensorGeometry

EXIT_OK = 0
EXIT_INPUT_ERROR = 2
"""Exit codes: plan written, input to be fixed."""
DEFAULT_SEED = 0
"""Master seed when --seed is not given."""
INDICATIVE_NOTE = (
    "no --sensor-config was given, so the INDICATIVE sensor geometry is used (640 x 480 px, f about 688 px, "
    "10 frames/s). These are not datasheet values: the field-of-view fit, the jitter in millimeters, "
    "the expected depth quantum and the Section 9 budget will change when the real values are known (Step 4.5).")
"""The loud note printed and written to the summary when the geometry is indicative."""


class PlanInputError(Exception):
    """An input the technician has to fix; the message says what to change."""


def build_parser() -> argparse.ArgumentParser:
    """The argument parser (every option has help text)."""
    parser = argparse.ArgumentParser(
        description="Plan the stations and poses of the characterization capture (Sections 4 to 9): registration, A "
                    "noise, B-HV edges, B-Z depth steps, C area, D detection, with logged randomization and drift "
                    "sentinels. Writes poses.csv, plan_summary.txt, plan.png, targets.json and parameters.json.")
    parser.add_argument("--out", required=True, type=Path, metavar="DIR", help="output directory (created if needed)")
    parser.add_argument("--registration", type=Path, metavar="PATH",
                        help="registration.json; adds the flange pose to command (robot base frame) to poses.csv")
    parser.add_argument("--sensor-config", type=Path, metavar="PATH",
                        help="sensor_config.json with the sensor geometry; without it the indicative geometry is used")
    parser.add_argument("--parameters", type=Path, metavar="PATH",
                        help="parameters.json with overrides of the Section 2 parameter table")
    parser.add_argument("--series", nargs="*", metavar="LETTER", default=None,
                        help="subset of series to plan, by procedure letter: " + " ".join(PLANNED_SERIES)
                             + " (R registration, A noise, B edges, Z depth steps, C area, D detection); default all. "
                               "Without letters (--series alone) no series is planned, which is meant for a plan of the "
                               "optional drift run alone (--drift-run)")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, metavar="N",
                        help="master random seed; every shuffle and offset draws from it and is logged (default %(default)s)")
    parser.add_argument("--no-extended", action="store_true",
                        help="skip the extended trials of the low point (D_5) of the D series at the farthest stations "
                             "(Section 8)")
    parser.add_argument("--filters-off", action="store_true",
                        help="append the filters-off repeat of the A and B series (A, B-HV and B-Z; Section 4, Step 4.2) "
                             "after each filters-on series; its poses are labeled filters_off and are listed outside "
                             "the main budget in plan_summary.txt, together with the drift sentinels captured "
                             "during the repeat")
    parser.add_argument("--staircase", action="store_true",
                        help="add the optional second pass of the B-Z series, the fine staircase (Section 6.2), at the "
                             "reduced stations; its poses are labeled staircase and are listed outside the main budget "
                             "in plan_summary.txt, with the drift sentinels captured during it (the ramp and the step ladder are "
                             "always planned). To use the disparity quantum "
                             "measured by the ramp instead of the assumed one (0.125 px), pass it as "
                             "tier_a_disparity_quantum_px in the --parameters JSON.")
    parser.add_argument("--reuse-c-first-frames", action="store_true",
                        help="let the first frame of each C pose of the same target, gap and station count as a D trial "
                             "(Section 8, Reuse: 30 of the 60 per configuration and station), so that the D main series "
                             "plans only the remaining poses; the Section 9 budget is still computed without the reuse "
                             "and plan_summary.txt says how many D poses were taken from C")
    parser.add_argument("--lateral-sweep", action="store_true",
                        help="add the optional second pass of the B-HV series (Section 6.1, Step 6): the edge target T3a "
                             "swept in H and then in V at Z_REFERENCE_MM in steps of LATERAL_SWEEP_STEP_PX over "
                             "LATERAL_SWEEP_SPAN_PX (20 poses per axis, approached from alternating directions); its poses "
                             "are labeled lateral_sweep and are listed outside the main budget in plan_summary.txt, with the "
                             "drift sentinels captured during the sweep")
    parser.add_argument("--drift-run", action="store_true",
                        help="add the optional separate drift run (Section 4, Step 3): T2 on a FIXED STAND at the reference "
                             "station, the robot idle, SENTINEL_FRAMES frames every DRIFT_RUN_CAPTURE_INTERVAL_MIN minutes for "
                             "DRIFT_RUN_DURATION_MIN minutes (241 captures with the default parameters; procedure S, "
                             "sub-series drift_run, pose indices from P4000). The captures are listed outside the main budget "
                             "in plan_summary.txt (count, frames, pose-index range, file names). Their poses.csv rows hold the "
                             "nominal pose of T2 at the reference station and the notes field fixed_stand=true: make_manifest "
                             "copies the robot pose columns from the nominal pose, so the pose log needs only timestamp and "
                             "sensor_temp_c for them (the registered pose is constant, and the analysis uses only the "
                             "relative mean Z). Give --series without letters to plan the run alone")
    parser.add_argument("--open-background", action="store_true",
                        help="add the open-background variant of the C series for the cutout arrays (Section 7, Step 4)")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the planner; returns the exit code."""
    args = build_parser().parse_args(argv)
    diagnostics = PlanDiagnostics(master_seed=args.seed)
    try:
        params = CharacterizationParameters.from_json(args.parameters) if args.parameters else CharacterizationParameters()
        if args.sensor_config is not None:
            geometry = SensorConfig.load(args.sensor_config).geometry
        else:
            geometry = SensorGeometry.indicative()
            print(f"NOTE: {INDICATIVE_NOTE}", file=sys.stderr)
            diagnostics.warn(INDICATIVE_NOTE)
        registration = Registration.load(args.registration) if args.registration else None
        targets = make_standard_target_set(params, geometry)
        plan = plan_full_session(params, geometry, np.random.default_rng(args.seed), registration=registration,
                                 filters_off=args.filters_off,
                                 open_background=args.open_background, extended=not args.no_extended, targets=targets,
                                 series=args.series, diagnostics=diagnostics, staircase=args.staircase,
                                 reuse_c_first_frames=args.reuse_c_first_frames, lateral_sweep=args.lateral_sweep,
                                 drift_run=args.drift_run)
        if not plan:
            raise PlanInputError("nothing to plan: --series was given without letters and --drift-run was not given; "
                                 "name the series to plan, or add --drift-run")
    except (OSError, ValueError, MissingSensorValue, PlanInputError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    written = write_plan(args.out, plan, registration, params, geometry, diagnostics)
    targets.save(args.out / TARGETS_FILE_NAME)
    params.to_json(args.out / PARAMETERS_FILE_NAME)
    print((args.out / PLAN_SUMMARY_NAME).read_text(encoding="utf-8"), end="")
    print("Wrote " + ", ".join(path.name for path in written) + f", {TARGETS_FILE_NAME} and {PARAMETERS_FILE_NAME} "
          f"in {args.out}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
