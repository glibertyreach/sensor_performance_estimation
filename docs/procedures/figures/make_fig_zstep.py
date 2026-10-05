"""
fig_zstep.png -- the two captures of the B-Z (depth step) series, drawn at the reference station.

Left panel: the step ladder at one Z0. The rungs are multiples of the expected depth quantum at the
station (Z_STEP_LADDER_QUANTA, each at least ROBOT_MIN_RESOLVABLE_MOVE_MM); the planner's own rule
(plan.z_step_rungs_mm) gives them in mm. For each rung the target alternates between Z0 (visit A) and
Z0 + rung (visit B) for Z_STEP_REPEATS cycles (A, B, A, B, ...). The vertical axis is the commanded
offset from Z0 on a symmetric-log scale; the horizontal axis is the visit number. Every visit is
captured with FRAMES_PER_ZSTEP_POSE frames; the analysis takes the truth of each step from the
read-back robot pose, not from the commanded step.

Right panel: the ramp. T2 is tilted about H by the small angle (plan.ramp_tilt_deg) that makes the true
depth across the visible plate span RAMP_QUANTA expected quanta, and captured once with
FRAMES_PER_RAMP_POSE frames. Each image row lies at one true depth (the straight line); a sensor that
quantizes depth at the expected quantum reports the staircase drawn over it (an illustration of what
the capture is expected to show, not a measurement). The expected quantum is q Z^2 / k with the
indicative disparity quantum of the Tier-A simulator (the measured q of analysis A replaces it).

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
from sensorperf.acquisition.plan import ramp_tilt_deg, ramp_visible_height_mm, z_step_rungs_mm  # noqa: E402
from sensorperf.geometry.targets import make_standard_target_set  # noqa: E402
from sensorperf.parameters import CharacterizationParameters, SensorGeometry, TARGET_NOISE_PLATE  # noqa: E402
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


def z0_mm() -> float:
    """The station drawn: the reference station, which is one of the reduced stations (400, 800, 1600 mm)."""
    return PARAMS.z_reference_mm


def station_rungs_mm() -> list[float]:
    quantum = GEOMETRY.depth_quantum_mm(INDICATIVE_QUANTUM_PX, z0_mm())
    return z_step_rungs_mm(PARAMS, quantum)


def ladder_visits(rungs) -> list[tuple[int, float, str, float]]:
    """(visit number, commanded offset mm, 'A' or 'B', rung delta) for the whole ladder, rung after rung."""
    visits = []
    number = 0
    for delta in rungs:
        for _ in range(PARAMS.z_step_repeats):
            for label, offset in (("A", 0.0), ("B", delta)):
                visits.append((number, offset, label, delta))
                number += 1
    return visits


def draw_ladder(ax) -> int:
    rungs = station_rungs_mm()
    visits = ladder_visits(rungs)
    for index, delta in enumerate(rungs):
        mine = [v for v in visits if v[3] == delta]
        color = RUNG_COLORS[index % len(RUNG_COLORS)]
        xs = [v[0] for v in mine]
        ys = [v[1] for v in mine]
        ax.step(xs, ys, where="post", color=color, lw=1.0)
        ax.text(xs[0] + 0.4, max(ys) * 1.35 + 0.004, f"{delta:.3g}", fontsize=7, color=color, va="bottom")
    ax.set_yscale("symlog", linthresh=SYMLOG_LINTHRESH_MM)
    ax.set_ylim(-0.004, rungs[-1] * 3.0)
    ax.set_xlim(-2, len(visits) + 1)
    ax.set_xlabel("visit number (A, B, A, B, ... for each step size in turn)", fontsize=9)
    ax.set_ylabel("commanded offset from Z0 (mm, symmetric-log scale)", fontsize=9)
    ax.set_title(f"(a) step ladder at Z0 = {z0_mm():g} mm: {len(rungs)} step sizes (labels, mm), {PARAMS.z_step_repeats} ABAB cycles each",
                 fontsize=9.5)
    ax.text(0.01, 0.98, f"every visit: {PARAMS.frames_per_zstep_pose} frames; rungs are multiples\n"
            f"({', '.join(f'{q:g}' for q in PARAMS.z_step_ladder_quanta)}) of the expected quantum",
            transform=ax.transAxes, ha="left", va="top", fontsize=7.5, color=GRAY)
    ax.grid(axis="y", color="#e5e5e5", lw=0.5)
    return len(rungs)


def draw_ramp(ax) -> None:
    z0 = z0_mm()
    quantum = GEOMETRY.depth_quantum_mm(INDICATIVE_QUANTUM_PX, z0)
    plate = make_standard_target_set(PARAMS, GEOMETRY).get(TARGET_NOISE_PLATE)
    tilt = ramp_tilt_deg(PARAMS, GEOMETRY, plate, z0, quantum)
    visible = ramp_visible_height_mm(GEOMETRY, plate, z0)
    span = PARAMS.ramp_quanta * quantum
    rows = [visible * (k / 400.0 - 0.5) for k in range(401)]       # position down the plate, mm from its center
    truth = [span * (r / visible + 0.5) for r in rows]             # true depth above the near edge, mm
    reported = [quantum * int(t // quantum) for t in truth]        # what a sensor quantizing at the expected quantum shows
    ax.plot(rows, truth, color=BLUE, lw=1.4, label="true depth of each image row (read-back pose)")
    ax.step(rows, reported, where="post", color=VERMILLION, lw=1.2, label="reported depth if the sensor quantizes (illustration)")
    for k in range(int(PARAMS.ramp_quanta) + 1):
        ax.axhline(k * quantum, color=GRAY, lw=0.5, linestyle=":")
    ax.set_xlabel("position down the visible plate (mm from its center)", fontsize=9)
    ax.set_ylabel("depth above the near edge (mm)", fontsize=9)
    ax.set_title(f"(b) ramp at Z0 = {z0:g} mm (one pose at every ladder station)", fontsize=9.5)
    ax.text(0.02, 0.97, f"T2 tilted about H by {tilt:.2f} degrees;\ndepth spans {PARAMS.ramp_quanta:g} expected quanta ({span:.1f} mm)\n"
            f"over the {visible:.0f} mm visible height; expected quantum {quantum:.2f} mm;\n"
            f"{PARAMS.frames_per_ramp_pose} frames, read-back pose logged",
            transform=ax.transAxes, fontsize=7.5, va="top", color=GRAY)
    ax.legend(loc="lower right", fontsize=7, frameon=False)


def main() -> None:
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI, gridspec_kw={"width_ratios": [1.5, 1.0]})
    rungs = draw_ladder(ax_a)
    draw_ramp(ax_b)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.92, bottom=0.13, wspace=0.18)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, zstep_rung_count=rungs)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
