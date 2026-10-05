"""
fig_procedure_flow.png -- the order of the five capture series and the six analyses of the
VSX3000 performance test, and what each one passes on to the others.

    captures (shaded):  R registration, A noise plate, B-HV edges, B-Z depth steps, C arrays, D trials
    analyses (open):    A, B-HV, B-Z, C, D, E

What the arrows say (specification Section 1):
    - Registration (R) gives every frame its ground-truth target pose, so it feeds every analysis.
    - Analysis A supplies the noise level that sets the detection thresholds of B, C and D.
    - Analysis B-HV supplies the edge spread function that predicts the area bias measured in C.
    - Analysis E needs no captures of its own; it reuses the B and C frames.
    - The D pilot comes from the quick-look detection count on C data (dashed): it sets the D levels.

Why draw it: the technician captures in the order A, B, C, D, and sees at once why that order is
fixed and why registration comes first.

    python3 docs/procedures/figures/make_fig_procedure_flow.py      (from the repository root)
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import figfacts  # noqa: E402
from sensorperf.parameters import CharacterizationParameters, FIELD_POSITION_CODES  # noqa: E402

PARAMS = CharacterizationParameters()

OUTPUT_DPI = 200
FIGURE_SIZE_IN = (13.0, 4.8)
FIGURE_NAME = "fig_procedure_flow"

# Okabe-Ito palette.
BLACK = "#000000"
ORANGE = "#E69F00"
SKY_BLUE = "#56B4E9"
BLUISH_GREEN = "#009E73"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
REDDISH_PURPLE = "#CC79A7"
GRAY = "#595959"

CAPTURE_FILL = SKY_BLUE
CAPTURE_FILL_ALPHA = 0.45
BOX_WIDTH = 2.1
BOX_HEIGHT = 0.8
COLUMN_X = {"R": 1.2, "A": 3.5, "B-HV": 5.8, "B-Z": 8.1, "C": 10.4, "D": 12.7, "E": 15.0}
"""Column centers of the six series and of E (figure units)."""
CAPTURE_Y = 4.25
ANALYSIS_Y = 1.75
REGISTRATION_BAR_Y = 0.45
NOISE_LANE_Y = 3.25
ESF_LANE_Y = 2.95
E_LANE_Y = 2.75
LANE_DROP_OFFSET = 0.5     # lane arrows land this far from the box center, so they do not hit the capture arrow
FONT_SIZE = 8

CAPTURES = [
    ("R", f"R  registration\nT1 at {PARAMS.registration_poses} poses"),
    ("A", f"A  noise plate\nT2, {len(PARAMS.noise_stations_mm())} Z x {len(FIELD_POSITION_CODES)} field pos."),
    ("B-HV", "B-HV  edges\nT3a, T3b, 2 gaps"),
    ("B-Z", "B-Z  depth steps\nT2 + dial indicator"),
    ("C", "C  area arrays\nT4, T5, 2 gaps"),
    ("D", f"D  detection trials\nT4, T5, {PARAMS.frames_per_detection_trial} frame per pose"),
]
ANALYSES = [
    ("A", "Analysis A\nnoise vs Z"),
    ("B-HV", "Analysis B-HV\nrise distance, MTF50"),
    ("B-Z", "Analysis B-Z\nsmallest detected step"),
    ("C", "Analysis C\ntrue vs sensed area"),
    ("D", "Analysis D\nD_50, D_10, D_0"),
    ("E", "Analysis E\nboundary bias"),
]
# Capture box counts and analysis box counts are emitted as facts for the verification gate.


def box(ax, key, y, text, filled):
    """A rounded box centered on its column; shaded for captures, open for analyses."""
    cx = COLUMN_X[key]
    patch = FancyBboxPatch((cx - BOX_WIDTH / 2, y - BOX_HEIGHT / 2), BOX_WIDTH, BOX_HEIGHT,
                           boxstyle="round,pad=0.02,rounding_size=0.08",
                           facecolor=CAPTURE_FILL if filled else "white", edgecolor=BLACK, lw=1.2,
                           alpha=CAPTURE_FILL_ALPHA if filled else 1.0)
    ax.add_patch(patch)
    if filled:      # redraw the edge opaque so the outline stays crisp under the alpha fill
        ax.add_patch(FancyBboxPatch((cx - BOX_WIDTH / 2, y - BOX_HEIGHT / 2), BOX_WIDTH, BOX_HEIGHT,
                                    boxstyle="round,pad=0.02,rounding_size=0.08", facecolor="none",
                                    edgecolor=BLACK, lw=1.2))
    ax.text(cx, y, text, ha="center", va="center", fontsize=FONT_SIZE)


def arrow(ax, start, end, color=BLACK, style="-|>", lw=1.3, dashed=False, rad=0.0):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=12, color=color, lw=lw,
                                 linestyle="--" if dashed else "-", connectionstyle=f"arc3,rad={rad}",
                                 shrinkA=0, shrinkB=0))


def bus(ax, source_key, lane_y, target_keys, color, label, label_x, source_offset=0.0):
    """A line from the top of an analysis box up to a lane, along the lane, and an arrowhead
    down into each target box top (offset from the center so it clears the capture arrow)."""
    top = ANALYSIS_Y + BOX_HEIGHT / 2
    sx = COLUMN_X[source_key] + source_offset
    xs = [COLUMN_X[k] - LANE_DROP_OFFSET for k in target_keys]
    ax.plot([sx, sx, max(xs)], [top, lane_y, lane_y], color=color, lw=1.3, solid_capstyle="butt")
    for x in xs:
        arrow(ax, (x, lane_y), (x, top), color=color)
    ax.text(label_x, lane_y + 0.05, label, fontsize=7.5, color=color, ha="left", va="bottom",
            bbox=dict(facecolor="white", edgecolor="none", pad=0.5))


def main() -> None:
    fig, ax = plt.subplots(figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI)
    top = ANALYSIS_Y + BOX_HEIGHT / 2
    bottom_capture = CAPTURE_Y - BOX_HEIGHT / 2

    for key, text in CAPTURES:
        box(ax, key, CAPTURE_Y, text, filled=True)
    # E has no captures of its own: a dotted placeholder in the capture row.
    ax.add_patch(FancyBboxPatch((COLUMN_X["E"] - BOX_WIDTH / 2, CAPTURE_Y - BOX_HEIGHT / 2), BOX_WIDTH, BOX_HEIGHT,
                                boxstyle="round,pad=0.02,rounding_size=0.08", facecolor="none", edgecolor=GRAY,
                                lw=1.0, linestyle=":"))
    ax.text(COLUMN_X["E"], CAPTURE_Y, "no captures:\nreuses the B and C frames", ha="center", va="center",
            fontsize=FONT_SIZE - 0.5, color=GRAY)
    for key, text in ANALYSES:
        box(ax, key, ANALYSIS_Y, text, filled=False)

    # Each capture series feeds its own analysis.
    for key in ("A", "B-HV", "B-Z", "C", "D"):
        arrow(ax, (COLUMN_X[key], bottom_capture), (COLUMN_X[key], top))

    # Registration feeds every analysis: a bar below the analyses with an arrow up into each.
    bar_left, bar_right = COLUMN_X["R"] - BOX_WIDTH / 2, COLUMN_X["E"] + BOX_WIDTH / 2
    ax.add_patch(FancyBboxPatch((bar_left, REGISTRATION_BAR_Y - 0.2), bar_right - bar_left, 0.4,
                                boxstyle="round,pad=0.02,rounding_size=0.08", facecolor=ORANGE, alpha=0.35,
                                edgecolor=BLACK, lw=1.2))
    ax.text((bar_left + bar_right) / 2, REGISTRATION_BAR_Y,
            "registration.json: robot-to-sensor transform, the ground-truth pose of the target in every frame",
            ha="center", va="center", fontsize=FONT_SIZE)
    ax.plot([COLUMN_X["R"], COLUMN_X["R"]], [bottom_capture, REGISTRATION_BAR_Y + 0.2], color=BLACK, lw=1.3)
    arrow(ax, (COLUMN_X["R"], bottom_capture), (COLUMN_X["R"], REGISTRATION_BAR_Y + 0.2))
    for key, _ in ANALYSES:
        arrow(ax, (COLUMN_X[key], REGISTRATION_BAR_Y + 0.2), (COLUMN_X[key], ANALYSIS_Y - BOX_HEIGHT / 2),
              color=ORANGE, lw=1.6)

    # A supplies the noise level (thresholds) to B-HV, B-Z, C and D.
    bus(ax, "A", NOISE_LANE_Y, ["B-HV", "B-Z", "C", "D"], BLUE,
        "noise level sets the thresholds of B, C and D", COLUMN_X["A"] + 0.6, source_offset=LANE_DROP_OFFSET)
    # B-HV supplies the edge spread function to C.
    bus(ax, "B-HV", ESF_LANE_Y, ["C"], BLUISH_GREEN, "edge spread function predicts the area bias",
        COLUMN_X["B-HV"] + 0.3, source_offset=0.2)
    # E reuses B and C data: lanes from B-HV and C into E.
    ex = COLUMN_X["E"] - LANE_DROP_OFFSET
    for source_key, offset in (("B-HV", 0.5), ("C", 0.5)):
        sx = COLUMN_X[source_key] + offset
        ax.plot([sx, sx, ex], [top, E_LANE_Y, E_LANE_Y], color=VERMILLION, lw=1.3)
    arrow(ax, (ex, E_LANE_Y), (ex, top), color=VERMILLION)
    ax.text(COLUMN_X["D"] + 0.2, E_LANE_Y - 0.28, "E reuses the data of B and C", fontsize=7.5, color=VERMILLION,
            ha="left", va="top")

    # The D pilot comes from the quick-look count on C data (dashed).
    top_capture = CAPTURE_Y + BOX_HEIGHT / 2
    arrow(ax, (COLUMN_X["C"] + 0.4, top_capture), (COLUMN_X["D"] - 0.4, top_capture), color=REDDISH_PURPLE,
          dashed=True, lw=1.6, rad=-0.6)
    ax.text((COLUMN_X["C"] + COLUMN_X["D"]) / 2, CAPTURE_Y + 0.9, "dashed: the pilot. A quick-look\ncount on C sets the D levels",
            fontsize=7.5, color=REDDISH_PURPLE, ha="center", va="bottom")

    # Capture order, left to right.
    ax.annotate("", xy=(COLUMN_X["D"] + BOX_WIDTH / 2, 5.85), xytext=(COLUMN_X["R"] - BOX_WIDTH / 2, 5.85),
                arrowprops=dict(arrowstyle="-|>", color=GRAY, lw=1.0))
    ax.text((COLUMN_X["R"] + COLUMN_X["D"]) / 2, 5.9, "capture order: R, A, B, C, D.   Shaded boxes are captures, open boxes are analyses.", ha="center", va="bottom",
            fontsize=8.5, color=GRAY)

    ax.set_xlim(0.0, 16.3)
    ax.set_ylim(0.0, 6.3)
    ax.axis("off")
    fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, capture_box_count=len(CAPTURES), analysis_box_count=len(ANALYSES))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
