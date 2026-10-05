"""
Command line: plan the stations and poses of the characterization capture
(procedure document, Sections 4 to 9).

    python3 -m sensorperf.cli.plan_stations --out plan/ --seed 1
    python3 -m sensorperf.cli.plan_stations --out plan/ --registration registration.json \\
        --sensor-config sensor_config.json --series A C --seed 7

What it computes
----------------
Every commanded pose of the series of Part I, in the order of the procedure:
the registration poses of Section 4 (R), the noise series of Section 5 (A:
main stations in a seeded random order, tilt sub-series, repeat-mount check,
optional filters-off repeat), the edge series of Section 6.1 (B), the Z-step
series of Section 6.2 (Z: step ladder and fine staircase), the area series of
Section 7 (C: jittered poses, field sub-series, optional open-background
variant) and the detection series of Section 8 (D: main and extended trials),
with drift sentinels inserted on the budget clock. Every random draw uses
``np.random.default_rng(seed)`` with a seed derived from ``--seed`` and logged in
poses.csv. A target that would not fit the field of view at its station is pulled
inward along its field direction and the summary says so (Section 5, Step 1).

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
    --parameters PATH     parameters.json with overrides of the Section 2 table.
    --pilot-d50-mm, --pilot-d0-mm   pilot D_50 and D_0 of the arrays (Section 8,
                          Step 1), as one number for every array and gap, or as
                          TARGET@GAP=VALUE entries (for example T4-S@15=1.2).

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
    AREA_TARGET_ORDER, PLAN_SUMMARY_NAME, PLANNED_SERIES, PlanDiagnostics, plan_full_session, write_plan,
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
PILOT_ENTRY_SEPARATOR = "="
PILOT_KEY_SEPARATOR = "@"
"""Syntax of a pilot entry: TARGET@GAP=VALUE."""
INDICATIVE_NOTE = (
    "no --sensor-config was given, so the INDICATIVE sensor geometry is used (640 x 480 px, f about 688 px, "
    "10 frames/s). These are not datasheet values: the field-of-view fit, the jitter in millimeters, "
    "the expected depth quantum and the Section 9 budget will change when the real values are known (Step 4.5).")
"""The loud note printed and written to the summary when the geometry is indicative."""


class PlanInputError(Exception):
    """An input the technician has to fix; the message says what to change."""


def parse_pilot_values(tokens: list[str] | None, option: str) -> dict | None:
    """The pilot dictionary keyed by (target_id, gap) from the command-line tokens: a bare number applies to every
    array and gap (stored under the key (None, None) and expanded by :func:`expand_pilot`); TARGET@GAP=VALUE sets one
    entry. Returns None when no token was given."""
    if not tokens:
        return None
    table: dict = {}
    for token in tokens:
        try:
            if PILOT_ENTRY_SEPARATOR in token:
                key_text, value_text = token.split(PILOT_ENTRY_SEPARATOR, 1)
                target, gap_text = key_text.split(PILOT_KEY_SEPARATOR, 1)
                table[(target.strip(), float(gap_text))] = float(value_text)
            else:
                table[(None, None)] = float(token)
        except ValueError:
            raise PlanInputError(f"{option}: {token!r} is not a number or a TARGET@GAP=VALUE entry (for example "
                                 "T4-S@15=1.2)") from None
    return table


def expand_pilot(table: dict | None, params: CharacterizationParameters) -> dict | None:
    """Expand a bare number to every array and both gaps; explicit entries win."""
    if table is None:
        return None
    expanded: dict = {}
    if (None, None) in table:
        for target in AREA_TARGET_ORDER:
            for gap in (params.gap_small_mm, params.gap_large_mm):
                expanded[(target, gap)] = table[(None, None)]
    expanded.update({k: v for k, v in table.items() if k != (None, None)})
    return expanded


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
    parser.add_argument("--series", nargs="+", metavar="LETTER", default=None,
                        help="subset of series to plan, by procedure letter: " + " ".join(PLANNED_SERIES)
                             + " (R registration, A noise, B edges, Z depth steps, C area, D detection); default all")
    parser.add_argument("--pilot-d50-mm", nargs="+", metavar="VALUE", default=None,
                        help="pilot D_50 in mm (Section 8, Step 1): one number for every array, or TARGET@GAP=VALUE "
                             "entries; used only for the summary and a warning")
    parser.add_argument("--pilot-d0-mm", nargs="+", metavar="VALUE", default=None,
                        help="pilot D_0 in mm, same syntax as --pilot-d50-mm")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, metavar="N",
                        help="master random seed; every shuffle and offset draws from it and is logged (default %(default)s)")
    parser.add_argument("--no-extended", action="store_true",
                        help="skip the extended 0 percent trials of the D series (Section 8, Step 4)")
    parser.add_argument("--filters-off", action="store_true",
                        help="append the filters-off repeat of the A series (Section 5, Step 7)")
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
        pilot_d50 = expand_pilot(parse_pilot_values(args.pilot_d50_mm, "--pilot-d50-mm"), params)
        pilot_d0 = expand_pilot(parse_pilot_values(args.pilot_d0_mm, "--pilot-d0-mm"), params)
        targets = make_standard_target_set(params, geometry)
        plan = plan_full_session(params, geometry, np.random.default_rng(args.seed), registration=registration,
                                 pilot_d50_mm=pilot_d50, pilot_d0_mm=pilot_d0, filters_off=args.filters_off,
                                 open_background=args.open_background, extended=not args.no_extended, targets=targets,
                                 series=args.series, diagnostics=diagnostics)
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
