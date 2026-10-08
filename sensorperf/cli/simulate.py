"""
Command line: write a synthetic characterization session.

    python3 -m sensorperf.cli.simulate --out DIR [--series A B Z C D R] [--quick] [--seed N] [--frame-scale F]

Builds the demonstration plan of :mod:`sensorperf.simulate.demo_plan` (every series, or the chosen
ones), renders it with the INDICATIVE sensor model of :mod:`sensorperf.simulate.sensor_model` and writes
the Section 9 session folder (frames, manifest.csv and the JSON records) with
:func:`sensorperf.simulate.session.write_synthetic_session`. ``--quick`` renders a 160 x 120 sensor with
the same field of view (the indicative geometry with fx, fy, cx, cy, width and height divided by 4) and
scales the frame count by 0.2; ``--frame-scale`` overrides the scale. Because the pixels of the quick sensor
are four times larger and its focal length four times smaller, the disparity noise, the disparity quantum and the
minimum feature diameter (the synthetic matcher's minimum detectable size, in pixels) are divided by the same factor
so that the depth noise, the depth quantum and the minimum feature size in mm are those of the full-size
sensor (sigma_Z = sigma_d Z^2 / k and delta_Z = q Z^2 / k both keep their values; see
``SyntheticSensorModel.indicative_scaled``). The physical targets are always the
standard set of the FULL-size indicative geometry, so a quick session uses the same targets as a full one
and merely sees them with larger pixels. The pixel width of the boundary band is divided by the same factor
(:func:`sensorperf.simulate.demo_plan.scaled_parameters`), so that it covers the same millimeters of the 200 x 150 mm T2
board, which is only 21 x 16 px on the quick sensor at 1600 mm.
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from typing import Sequence

import numpy as np

from sensorperf.geometry.targets import make_standard_target_set
from sensorperf.io.manifest import load_manifest, MANIFEST_FILE_NAME
from sensorperf.parameters import CharacterizationParameters, SensorGeometry
from sensorperf.simulate.demo_plan import ALL_SERIES, demo_plan, demo_registration, scaled_geometry, scaled_parameters
from sensorperf.simulate.sensor_model import SyntheticSensorModel
from sensorperf.simulate.session import write_synthetic_session

DEFAULT_SEED = 20261005
"""Default seed of the random generator (plan draws, noise, robot repeatability)."""
QUICK_FRAME_SCALE = 0.2
"""Frame scale of --quick."""
QUICK_PIXEL_DIVISOR = 4
"""--quick divides the image size and the pixel intrinsics by this (640 x 480 -> 160 x 120)."""
DEMO_ROBOT_REPEATABILITY_MM = 0.05
"""Robot repeatability (per axis, Gaussian) of the demonstration session."""
DEMO_DRIFT_MM_PER_HOUR = 1.0
"""Depth drift of the demonstration model, so that the two drift sentinels differ visibly."""


def build_parser() -> argparse.ArgumentParser:
    """The argument parser of this command."""
    parser = argparse.ArgumentParser(
        prog="python3 -m sensorperf.cli.simulate",
        description="Write a synthetic (INDICATIVE) characterization session with a demonstration plan.")
    parser.add_argument("--out", required=True, help="session folder to write")
    parser.add_argument("--series", nargs="+", choices=ALL_SERIES, default=list(ALL_SERIES),
                        help="series to simulate (default: all of %(choices)s)")
    parser.add_argument("--quick", action="store_true",
                        help=f"160 x 120 sensor with the same field of view and {QUICK_FRAME_SCALE} of the frames")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="seed of the random generator")
    parser.add_argument("--frame-scale", type=float, default=None,
                        help="fraction of the planned frames per pose to render (default 1, or 0.2 with --quick)")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the command; returns the process exit code."""
    arguments = build_parser().parse_args(argv)
    full_geometry = SensorGeometry.indicative()
    geometry = scaled_geometry(full_geometry, QUICK_PIXEL_DIVISOR) if arguments.quick else full_geometry
    params = scaled_parameters(CharacterizationParameters(), QUICK_PIXEL_DIVISOR if arguments.quick else 1)
    frame_scale = (arguments.frame_scale if arguments.frame_scale is not None
                   else (QUICK_FRAME_SCALE if arguments.quick else 1.0))
    rng = np.random.default_rng(arguments.seed)
    pixel_divisor = QUICK_PIXEL_DIVISOR if arguments.quick else 1
    model = replace(SyntheticSensorModel.indicative_scaled(geometry, pixel_divisor),
                    fixed_pattern_seed=arguments.seed, drift_mm_per_hour=DEMO_DRIFT_MM_PER_HOUR)
    targets = make_standard_target_set(params, full_geometry)
    plan = demo_plan(params, geometry, rng, arguments.quick, series=arguments.series,
                     disparity_quantum_px=model.disparity_quantum_px)

    started = time.time()
    last_percent = -1

    def progress(index: int, total: int) -> None:
        """Print a progress line when the integer percentage changes."""
        nonlocal last_percent
        percent = 100 * index // total
        if percent != last_percent and (percent % 10 == 0 or index == total):
            last_percent = percent
            print(f"  {index}/{total} poses ({percent}%), {time.time() - started:.0f} s", flush=True)

    root = write_synthetic_session(arguments.out, params, geometry, model, demo_registration(), targets, plan, rng,
                                   frame_scale=frame_scale, progress=progress,
                                   robot_repeatability_mm=DEMO_ROBOT_REPEATABILITY_MM)
    frames = len(load_manifest(root / MANIFEST_FILE_NAME))
    print(f"Wrote synthetic session to {root}: {len(plan)} poses, {frames} frames "
          f"({geometry.image_width_px} x {geometry.image_height_px} px) in {time.time() - started:.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
