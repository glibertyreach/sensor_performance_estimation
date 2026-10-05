"""
fig_data_flow.png -- the session folder (left), the six analyses (middle) and what they write (right),
converging on forward_model_parameters.json.

The sub-folders and the session-wide files are read from the code (sensorperf.io.session.SERIES_DIRS and
the file-name constants), so a renamed folder cannot leave this figure behind. Which folder feeds which
analysis follows the procedure:
    A      A_noise (and the sentinels, for the drift correction)
    B-HV   B_edges
    B-Z    B_zstep
    C      C_area
    D      D_detect (the first frame of each C pose may also count as a D trial)
    E      B_edges and C_area (no captures of its own)
Every analysis also reads the session-wide files: sensor_config.json, registration.json,
targets_asbuilt.csv, targets.json, parameters.json and manifest.csv. The fits of A and the boundary
terms of E go into forward_model_parameters.json (specification Sections 10 and 14).

    python3 docs/procedures/figures/make_fig_data_flow.py      (from the repository root)
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
from sensorperf.geometry.registration import REGISTRATION_FILE_NAME  # noqa: E402
from sensorperf.geometry.targets import ASBUILT_FILE_NAME, TARGETS_FILE_NAME  # noqa: E402
from sensorperf.io.manifest import MANIFEST_FILE_NAME  # noqa: E402
from sensorperf.io.session import (  # noqa: E402
    ANALYSIS_DIR_NAME, FORWARD_MODEL_FILE_NAME, PARAMETERS_FILE_NAME, SENSOR_CONFIG_FILE_NAME, SERIES_DIRS,
)
from sensorperf.parameters import (  # noqa: E402
    PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES, PROCEDURE_NOISE, PROCEDURE_REGISTRATION,
    PROCEDURE_SENTINEL, PROCEDURE_ZSTEP,
)

OUTPUT_DPI = 200
FIGURE_SIZE_IN = (13.0, 6.2)
FIGURE_NAME = "fig_data_flow"

# Okabe-Ito palette.
BLACK = "#000000"
ORANGE = "#E69F00"
SKY_BLUE = "#56B4E9"
BLUISH_GREEN = "#009E73"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
GRAY = "#595959"

FOLDER_X, ANALYSIS_X, OUTPUT_X, MODEL_X = 1.7, 6.6, 10.4, 14.0
FOLDER_W, ANALYSIS_W, OUTPUT_W, MODEL_W = 2.9, 2.5, 3.5, 2.4
BOX_H = 0.62
FONT = 8

ANALYSES = ["A", "B-HV", "B-Z", "C", "D", "E"]
ANALYSIS_Y = {"A": 5.0, "B-HV": 4.0, "B-Z": 3.0, "C": 2.0, "D": 1.0, "E": 0.0}
ANALYSIS_TEXT = {"A": "Analysis A: noise vs Z", "B-HV": "Analysis B-HV: H, V resolution", "B-Z": "Analysis B-Z: Z resolution",
                 "C": "Analysis C: true vs sensed area", "D": "Analysis D: D_50, D_10, D_0", "E": "Analysis E: boundary bias"}
OUTPUTS = {"A": "A_noise_summary.csv + figures", "B-HV": "B-HV tables + figures", "B-Z": "B-Z tables + figures",
           "C": "C_area_summary.csv + figures", "D": "D_detect_summary.csv + figures",
           "E": "E_boundary_bias.csv + figures"}
FOLDER_ORDER = [(PROCEDURE_NOISE, 5.3), (PROCEDURE_SENTINEL, 4.55), (PROCEDURE_EDGES, 3.6), (PROCEDURE_ZSTEP, 3.0),
                (PROCEDURE_AREA, 2.0), (PROCEDURE_DETECTION, 1.0)]
FOLDER_FEEDS = {PROCEDURE_NOISE: ["A"], PROCEDURE_SENTINEL: ["A"], PROCEDURE_EDGES: ["B-HV", "E"],
                PROCEDURE_ZSTEP: ["B-Z"], PROCEDURE_AREA: ["C", "E", "D"], PROCEDURE_DETECTION: ["D"]}
SESSION_FILES = (SENSOR_CONFIG_FILE_NAME, REGISTRATION_FILE_NAME, ASBUILT_FILE_NAME, TARGETS_FILE_NAME,
                 PARAMETERS_FILE_NAME, MANIFEST_FILE_NAME)


def box(ax, cx, cy, w, h, text, face="white", alpha=1.0, edge=BLACK, fontsize=FONT, lw=1.1):
    ax.add_patch(FancyBboxPatch((cx - w / 2, cy - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                facecolor=face, edgecolor=edge, lw=lw, alpha=alpha))
    ax.text(cx, cy, text, ha="center", va="center", fontsize=fontsize)


def arrow(ax, start, end, color=GRAY, lw=1.0, rad=0.0):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=9, color=color, lw=lw,
                                 connectionstyle=f"arc3,rad={rad}", shrinkA=0, shrinkB=0))


def main() -> None:
    fig, ax = plt.subplots(figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI)
    # Session-wide files (top), read by every analysis.
    box(ax, FOLDER_X, 6.85, FOLDER_W + 0.6, 1.0, "session-wide files, read by every analysis:\n" +
        "\n".join(", ".join(SESSION_FILES[i:i + 2]) for i in range(0, len(SESSION_FILES), 2)),
        face=ORANGE, alpha=0.30, fontsize=7)
    ax.plot([ANALYSIS_X - ANALYSIS_W / 2 - 0.35] * 2, [6.85, ANALYSIS_Y["E"]], color=ORANGE, lw=2.0)
    ax.plot([FOLDER_X + FOLDER_W / 2 + 0.3, ANALYSIS_X - ANALYSIS_W / 2 - 0.35], [6.85, 6.85], color=ORANGE, lw=2.0)
    for name in ANALYSES:
        arrow(ax, (ANALYSIS_X - ANALYSIS_W / 2 - 0.35, ANALYSIS_Y[name]), (ANALYSIS_X - ANALYSIS_W / 2, ANALYSIS_Y[name]),
              color=ORANGE, lw=1.6)
    # Capture folders.
    ax.text(FOLDER_X, 5.95, "capture folders", ha="center", fontsize=8.5, color=GRAY)
    registration_dir = SERIES_DIRS[PROCEDURE_REGISTRATION]
    for procedure, y in FOLDER_ORDER:
        box(ax, FOLDER_X, y, FOLDER_W, BOX_H - 0.1, SERIES_DIRS[procedure] + "/", face=SKY_BLUE, alpha=0.45)
    box(ax, FOLDER_X, 0.15, FOLDER_W, BOX_H - 0.1, registration_dir + "/", face=SKY_BLUE, alpha=0.45)
    ax.text(FOLDER_X, -0.35, f"feeds the registration solve, which writes {REGISTRATION_FILE_NAME}", ha="center",
            va="center", fontsize=7, color=GRAY)
    for procedure, y in FOLDER_ORDER:
        for target in FOLDER_FEEDS[procedure]:
            arrow(ax, (FOLDER_X + FOLDER_W / 2, y), (ANALYSIS_X - ANALYSIS_W / 2 - 0.35 + 0.0, ANALYSIS_Y[target]),
                  color=GRAY, lw=0.9, rad=0.0)
    # Analyses and their outputs.
    for name in ANALYSES:
        y = ANALYSIS_Y[name]
        box(ax, ANALYSIS_X, y, ANALYSIS_W, BOX_H, ANALYSIS_TEXT[name])
        box(ax, OUTPUT_X, y, OUTPUT_W, BOX_H - 0.1, f"{ANALYSIS_DIR_NAME}/{OUTPUTS[name]}", face="#f2f2f2", fontsize=7.5)
        arrow(ax, (ANALYSIS_X + ANALYSIS_W / 2, y), (OUTPUT_X - OUTPUT_W / 2, y), color=BLACK, lw=1.2)
    # forward_model_parameters.json
    model_y = (ANALYSIS_Y["A"] + ANALYSIS_Y["E"]) / 2
    box(ax, MODEL_X, model_y, MODEL_W, 1.5, f"{ANALYSIS_DIR_NAME}/\n{FORWARD_MODEL_FILE_NAME}\n\nhand-off to the Tier-A\nforward-noise simulator",
        face=BLUISH_GREEN, alpha=0.30, fontsize=8)
    arrow(ax, (OUTPUT_X + OUTPUT_W / 2, ANALYSIS_Y["A"]), (MODEL_X - MODEL_W / 2, model_y + 0.45), color=BLUE, lw=1.6, rad=-0.1)
    arrow(ax, (OUTPUT_X + OUTPUT_W / 2, ANALYSIS_Y["E"]), (MODEL_X - MODEL_W / 2, model_y - 0.45), color=VERMILLION, lw=1.6, rad=0.1)
    ax.text(OUTPUT_X + OUTPUT_W / 2 + 0.05, ANALYSIS_Y["A"] + 0.28, "sigma_d, sigma_0, q, k,\ncorrelation length", fontsize=7, color=BLUE, ha="left", va="bottom")
    ax.text(OUTPUT_X + OUTPUT_W / 2 + 0.05, ANALYSIS_Y["E"] - 0.28, "W_fab, W_drop, pi_near", fontsize=7, color=VERMILLION, ha="left", va="top")
    ax.text(ANALYSIS_X + 0.7, 5.95, "analyses (python3 -m sensorperf.cli.analyze)", ha="center", fontsize=8.5, color=GRAY)
    ax.text(OUTPUT_X, 5.95, "outputs", ha="center", fontsize=8.5, color=GRAY)
    ax.set_xlim(-0.2, 15.6)
    ax.set_ylim(-0.7, 7.5)
    ax.axis("off")
    fig.subplots_adjust(left=0.005, right=0.995, top=0.995, bottom=0.005)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, series_folder_count=len(SERIES_DIRS), analysis_count=len(ANALYSES))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
