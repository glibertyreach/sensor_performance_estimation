"""
fig_zstep.png -- the order of visits of the B-Z (depth step) series, at one Z0.

Left panel: the step ladder. For each step size delta of Z_STEP_LADDER_MM the target alternates
between Z0 (visit A) and Z0 + delta (visit B) for Z_STEP_REPEATS cycles (A, B, A, B, ...). The
vertical axis is the commanded offset from Z0 on a symmetric-log scale (the rungs span more than
two decades); the horizontal axis is the visit number. Every visit gets a dial-indicator reading
(circle), logged with FRAMES_PER_ZSTEP_POSE frames: the analysis uses the reading, not the commanded
step, as the truth.

Right panel: the fine staircase. The target is swept from Z0 to Z0 + Z_STAIRCASE_QUANTA expected
depth quanta in steps of one quantum / Z_STAIRCASE_SUBDIVISION, with Z_STAIRCASE_FRAMES frames per step
and the indicator logged at each. The expected quantum is q Z^2 / k with the indicative disparity
quantum of the Tier-A simulator (the measured q of analysis A replaces it once it exists).

    python3 docs/procedures/figures/make_fig_zstep.py      (from the repository root)
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import figfacts  # noqa: E402
from sensorperf.parameters import CharacterizationParameters, SensorGeometry  # noqa: E402
from sensorperf.simulate.sensor_model import SyntheticSensorModel  # noqa: E402

PARAMS = CharacterizationParameters()
GEOMETRY = SensorGeometry.indicative()
INDICATIVE_QUANTUM_PX = SyntheticSensorModel.indicative(GEOMETRY).disparity_quantum_px

OUTPUT_DPI = 200
FIGURE_SIZE_IN = (13.0, 4.5)
FIGURE_NAME = "fig_zstep"

# Okabe-Ito palette.
BLACK = "#000000"
ORANGE = "#E69F00"
SKY_BLUE = "#56B4E9"
BLUISH_GREEN = "#009E73"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
GRAY = "#595959"

SYMLOG_LINTHRESH_MM = 0.01
"""Symmetric-log linear region: the A visits (offset 0) sit on the linear part."""
RUNG_COLORS = (BLUE, SKY_BLUE, BLUISH_GREEN, ORANGE, VERMILLION, "#CC79A7", GRAY, BLACK)


def ladder_visits() -> list[tuple[int, float, str, float]]:
    """(visit number, commanded offset mm, 'A' or 'B', rung delta) for the whole ladder, rung after rung."""
    visits = []
    number = 0
    for delta in PARAMS.z_step_ladder_mm:
        for _ in range(PARAMS.z_step_repeats):
            for label, offset in (("A", 0.0), ("B", delta)):
                visits.append((number, offset, label, delta))
                number += 1
    return visits


def draw_ladder(ax) -> int:
    visits = ladder_visits()
    for index, delta in enumerate(PARAMS.z_step_ladder_mm):
        mine = [v for v in visits if v[3] == delta]
        color = RUNG_COLORS[index % len(RUNG_COLORS)]
        xs = [v[0] for v in mine]
        ys = [v[1] for v in mine]
        ax.step(xs, ys, where="post", color=color, lw=1.0)
        ax.plot(xs, ys, linestyle="none", marker="o", ms=2.6, mfc="white", mec=color, mew=0.7)
        ax.text(xs[0] + 0.4, max(ys) * 1.35 + 0.004, f"{delta:g}", fontsize=7, color=color, va="bottom")
    ax.set_yscale("symlog", linthresh=SYMLOG_LINTHRESH_MM)
    ax.set_ylim(-0.004, PARAMS.z_step_ladder_mm[-1] * 3.0)
    ax.set_xlim(-2, len(visits) + 1)
    ax.set_xlabel("visit number (A, B, A, B, ... for each step size in turn)", fontsize=9)
    ax.set_ylabel("commanded offset from Z0 (mm, symmetric-log scale)", fontsize=9)
    ax.set_title(f"(a) step ladder: {len(PARAMS.z_step_ladder_mm)} step sizes (labels, mm), {PARAMS.z_step_repeats} ABAB cycles each",
                 fontsize=9.5)
    ax.text(0.99, 0.03, f"every visit: {PARAMS.frames_per_zstep_pose} frames and one dial-indicator reading (circle)",
            transform=ax.transAxes, ha="right", fontsize=7.5, color=GRAY)
    ax.grid(axis="y", color="#e5e5e5", lw=0.5)
    return len(PARAMS.z_step_ladder_mm)


def draw_staircase(ax) -> None:
    z0 = PARAMS.z_reference_mm
    quantum = GEOMETRY.depth_quantum_mm(INDICATIVE_QUANTUM_PX, z0)
    step = quantum / PARAMS.z_staircase_subdivision
    count = int(round(PARAMS.z_staircase_quanta * PARAMS.z_staircase_subdivision)) + 1
    offsets = [index * step for index in range(count)]
    visit = list(range(count))
    ax.step(visit, offsets, where="post", color=BLUE, lw=1.2)
    ax.plot(visit, offsets, linestyle="none", marker="o", ms=4.0, mfc="white", mec=BLUE, mew=1.0)
    for k in range(int(PARAMS.z_staircase_quanta) + 1):
        ax.axhline(k * quantum, color=GRAY, lw=0.6, linestyle=":")
        ax.text(count - 0.5, k * quantum, f"{k} quantum" if k == 1 else (f"{k} quanta" if k else "Z0"), fontsize=7,
                color=GRAY, va="bottom", ha="right")
    ax.set_xlabel("step number (one visit per step)", fontsize=9)
    ax.set_ylabel("commanded offset from Z0 (mm)", fontsize=9)
    ax.set_title(f"(b) fine staircase at Z0 = {z0:g} mm",
                 fontsize=9.5)
    ax.text(0.02, 0.88, f"steps of one expected quantum / {PARAMS.z_staircase_subdivision};\nexpected quantum q Z^2 / k = {quantum:.2f} mm (indicative q);\n"
            f"{PARAMS.z_staircase_frames} frames and one indicator reading (circle) per step",
            transform=ax.transAxes, fontsize=7.5, va="top", color=GRAY)


def main() -> None:
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI, gridspec_kw={"width_ratios": [1.5, 1.0]})
    rungs = draw_ladder(ax_a)
    draw_staircase(ax_b)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.92, bottom=0.13, wspace=0.18)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, zstep_rung_count=rungs)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
