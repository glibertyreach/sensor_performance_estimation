"""
Command line: run the Part II analyses on a session folder.

    python3 -m sensorperf.cli.analyze --session Characterization_20261014/ [--only A B Z C D E]

Each analysis reads the session (manifest, registration, targets with the
as-built values, sensor configuration, parameters), selects its frames by the
manifest's procedure letter, and writes its outputs into ``<session>/analysis/``:
a CSV summary, a JSON detail file and figures (PNG and SVG). After all selected
analyses, ``forward_model_parameters.json`` is assembled from their results.

Letters: A noise (Section 10), B lateral resolution (Section 11.1), Z depth
resolution (Section 11.2), C area fidelity (Section 12), D detectability
(Section 13), E boundary bias (Section 14; reuses B and C data). An analysis
whose frames are absent from the manifest is skipped with a notice. Running
order is the document's: A, B, Z, C, D, E, because A supplies the noise level
B, C and D use, and B supplies the edge spread function C compares with.

Exit code: 0 when every selected analysis ran, 1 when one failed (its traceback
is printed and the others still run), 2 when the session cannot be read.
"""
from __future__ import annotations

import argparse
import importlib
import sys
import traceback
from pathlib import Path

from sensorperf.analysis.forward_model import write_forward_model
from sensorperf.io.session import Session

EXIT_OK = 0
EXIT_ANALYSIS_FAILED = 1
EXIT_INPUT_ERROR = 2
"""Exit codes: all ran, one failed, session unreadable."""

ANALYSES = (
    ("A", "sensorperf.analysis.noise", "run_noise"),
    ("B", "sensorperf.analysis.resolution_lateral", "run_lateral_resolution"),
    ("Z", "sensorperf.analysis.resolution_depth", "run_depth_resolution"),
    ("C", "sensorperf.analysis.area", "run_area"),
    ("D", "sensorperf.analysis.detection", "run_detection"),
    ("E", "sensorperf.analysis.boundary", "run_boundary_bias"),
)
"""(letter, module, entry function) in running order. Each entry function has the
signature run_x(session, out_dir, previous: dict) -> result, where ``previous`` maps
the letters already run to their results (A's noise level for B, C, D; B and C for E),
and each module has write_outputs(result, out_dir)."""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the Part II analyses (A, B, Z, C, D, E) on a session folder "
                                                 "and assemble forward_model_parameters.json.")
    parser.add_argument("--session", required=True, type=Path, metavar="DIR", help="session folder (Section 9 layout)")
    parser.add_argument("--only", nargs="+", choices=[a[0] for a in ANALYSES], metavar="LETTER",
                        help="run only these analyses (default: all, in the order A B Z C D E)")
    parser.add_argument("--out", type=Path, metavar="DIR",
                        help="output folder (default: <session>/analysis)")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        session = Session.load(args.session)
    except (OSError, ValueError, KeyError) as error:
        print(f"ERROR: cannot read the session {args.session}: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    out_dir = args.out if args.out is not None else session.analysis_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    chosen = set(args.only) if args.only else {a[0] for a in ANALYSES}
    results: dict = {}
    failed = False
    for letter, module_name, entry in ANALYSES:
        if letter not in chosen:
            continue
        module = importlib.import_module(module_name)
        print(f"== Analysis {letter} ({module_name})")
        try:
            result = getattr(module, entry)(session, out_dir, results)
            if result is None:
                print(f"   skipped: no {letter} frames in the manifest")
                continue
            module.write_outputs(result, out_dir)
            results[letter] = result
        except Exception:                                      # one failure must not stop the others
            failed = True
            traceback.print_exc()
            print(f"   analysis {letter} FAILED (see the traceback above); continuing", file=sys.stderr)
    if results:
        path = write_forward_model(results, out_dir, session.root, session.sensor.config_id)
        print(f"wrote {path}")
    return EXIT_ANALYSIS_FAILED if failed else EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
