"""Regenerate all VSX3000 performance-testing shop drawings that are drawn by the part scripts in this folder.

Usage (from anywhere):

    python3 docs/procedures/drawings/make_all.py

The PNG files are written next to this script.  The script prints, for every drawing, the number of layout
problems found by the automatic overlap check (text against text, text against lines, text outside the
border, title-block text outside its cell) and exits with status 1 if any were found.  Each part script also
prints its own geometry checks (collisions between holes, stack-ups against neighboring parts).
"""

from __future__ import annotations

import os
import sys

# Make the sibling modules importable regardless of the working directory.
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import part_01_target_adapter  # noqa: E402
import part_02_target_spigot  # noqa: E402
import part_03_standoffs  # noqa: E402
import part_04_t3a_raised_square  # noqa: E402
import part_05_t3b_square_window  # noqa: E402
import part_06_t4_disk_plate  # noqa: E402
import part_07_t5_cutout_plate  # noqa: E402


def main() -> int:
    """Draw all sheets; return the process exit code."""
    results: dict[str, list[str]] = {}
    results[part_01_target_adapter.NUMBER] = part_01_target_adapter.build(HERE)
    results[part_02_target_spigot.NUMBER] = part_02_target_spigot.build(HERE)
    results[part_03_standoffs.NUMBER] = part_03_standoffs.build(HERE)
    # PT-04 to PT-07 (target plates): each prints its geometry checks, then draws its sheet(s)
    for number, module in (("PT-04", part_04_t3a_raised_square), ("PT-05", part_05_t3b_square_window),
                           ("PT-06", part_06_t4_disk_plate), ("PT-07", part_07_t5_cutout_plate)):
        module.print_checks()
        results[number] = module.build(HERE)
    problems = 0
    for number in sorted(results):
        issues = results[number]
        print(f"{number}: {len(issues)} layout problem(s)")
        for issue in issues:
            print(f"    {issue}")
        problems += len(issues)
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
