"""
fig_stations.png -- where the targets are put: the Z stations (left) and the positions in the image
(right).

Left panel: the Z axis from Z_MIN to Z_MAX with three rows of ticks: the noise stations of series A
(Z_MIN to Z_MAX in steps of Z_NOISE_STEP_MM), the shape stations of B, C and D, and the reduced
stations used for the slow tests (B-Z, the D 0 percent series, the A tilt sub-series).

Right panel: the image rectangle of the indicative sensor geometry with the five field positions
(code 0 at the center, codes 1 to 4 at the corners, offset by FIELD_OFFSET_FRACTION of the half
field), and the square span of the random lateral offsets (PHASE_JITTER_SPAN_PX) at one corner. The
span is 8 px wide on a 640 px image, so it is shown true size and again magnified in an inset
with the pixel grid.

Why draw it: the technician sees at a glance which Z values and which image positions each series
visits, and how small the phase-jitter span is.

    python3 docs/procedures/figures/make_fig_stations.py      (from the repository root)
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import figfacts  # noqa: E402
from sensorperf.parameters import (  # noqa: E402
    CharacterizationParameters, FIELD_POSITION_CENTER, FIELD_POSITION_CODES, FIELD_POSITION_SIGNS, SensorGeometry,
)

PARAMS = CharacterizationParameters()
GEOMETRY = SensorGeometry.indicative()

OUTPUT_DPI = 200
FIGURE_SIZE_IN = (13.0, 4.5)
FIGURE_NAME = "fig_stations"

# Okabe-Ito palette.
BLACK = "#000000"
ORANGE = "#E69F00"
SKY_BLUE = "#56B4E9"
BLUISH_GREEN = "#009E73"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
REDDISH_PURPLE = "#CC79A7"
GRAY = "#595959"

ROW_Y = {"noise": 3.0, "shape": 2.0, "reduced": 1.0}
TICK_HALF_HEIGHT = 0.28
INSET_PIXELS = 4            # the inset shows this many pixels either side of the offset square
FIELD_MARKER_SIZE = 9


def draw_z_stations(ax) -> tuple[int, int, int]:
    noise = PARAMS.noise_stations_mm()
    shape = PARAMS.z_shape_stations_mm
    reduced = PARAMS.z_reduced_stations_mm
    rows = (("noise", noise, BLUE, "A: noise stations"),
            ("shape", shape, BLUISH_GREEN, "B, C, D: shape stations"),
            ("reduced", reduced, VERMILLION, "slow tests: reduced stations"))
    for key, values, color, label in rows:
        y = ROW_Y[key]
        ax.plot([PARAMS.z_min_mm, PARAMS.z_max_mm], [y, y], color=GRAY, lw=0.8)
        for z in values:
            ax.plot([z, z], [y - TICK_HALF_HEIGHT, y + TICK_HALF_HEIGHT], color=color, lw=2.4)
        ax.text(PARAMS.z_min_mm - 18, y, f"{label}\n({len(values)} stations)", ha="right", va="center",
                fontsize=8.5, color=color)
    for z in shape:
        ax.text(z, ROW_Y["shape"] - TICK_HALF_HEIGHT - 0.05, f"{z:g}", ha="center", va="top", fontsize=7, color=BLUISH_GREEN)
    for z in reduced:
        ax.text(z, ROW_Y["reduced"] - TICK_HALF_HEIGHT - 0.05, f"{z:g}", ha="center", va="top", fontsize=7, color=VERMILLION)
    for z in noise:
        ax.text(z, ROW_Y["noise"] + TICK_HALF_HEIGHT + 0.05, f"{z:g}", ha="center", va="bottom", fontsize=6.5, color=BLUE)
    ax.set_xlim(PARAMS.z_min_mm - 330, PARAMS.z_max_mm + 40)
    ax.set_xticks(range(int(PARAMS.z_min_mm), int(PARAMS.z_max_mm) + 1, 100))
    ax.set_ylim(0.2, 3.8)
    ax.set_xlabel("Z (mm), along the left IR camera's optical axis", fontsize=9)
    ax.set_yticks([])
    ax.spines["bottom"].set_bounds(PARAMS.z_min_mm, PARAMS.z_max_mm)
    for side in ("left", "right", "top"):
        ax.spines[side].set_visible(False)
    ax.set_title("(a) Z stations", fontsize=10)
    return len(noise), len(shape), len(reduced)


def field_positions_px() -> dict[int, tuple[float, float]]:
    """Pixel (column, row) of each field position: the image center, and the four corners offset by
    FIELD_OFFSET_FRACTION of the half field (sign pattern FIELD_POSITION_SIGNS)."""
    width, height = GEOMETRY.image_width_px, GEOMETRY.image_height_px
    center = (width / 2.0, height / 2.0)
    positions = {FIELD_POSITION_CENTER: center}
    for code, (sign_h, sign_v) in FIELD_POSITION_SIGNS.items():
        positions[code] = (center[0] + sign_h * PARAMS.field_offset_fraction * width / 2.0,
                           center[1] + sign_v * PARAMS.field_offset_fraction * height / 2.0)
    return positions


def draw_field_positions(ax) -> int:
    width, height = GEOMETRY.image_width_px, GEOMETRY.image_height_px
    ax.add_patch(Rectangle((0, 0), width, height, facecolor="white", edgecolor=BLACK, lw=1.4))
    positions = field_positions_px()
    for code in FIELD_POSITION_CODES:
        col, row = positions[code]
        ax.plot([col], [row], marker="o", ms=FIELD_MARKER_SIZE, color=ORANGE if code else SKY_BLUE, mec=BLACK, mew=0.8)
        ax.text(col, row, str(code), ha="center", va="center", fontsize=7, color=BLACK)
    ax.text(width / 2.0 + 14, height / 2.0 + 30, "center", fontsize=8, va="top")
    # The principal point is not the image center; mark nothing here, the stations are image-centered.
    # Dotted guide of the half-field fraction.
    half_w, half_h = PARAMS.field_offset_fraction * width / 2.0, PARAMS.field_offset_fraction * height / 2.0
    ax.add_patch(Rectangle((width / 2.0 - half_w, height / 2.0 - half_h), 2 * half_w, 2 * half_h, facecolor="none",
                           edgecolor=GRAY, lw=0.8, linestyle=":"))
    ax.text(width / 2.0 - half_w, height / 2.0 - half_h - 8, f"corners at {PARAMS.field_offset_fraction:g} of the half field",
            fontsize=7.5, color=GRAY, va="bottom")
    # Phase-jitter span at the corner of code 3 (+, +), true size.
    col, row = positions[3]
    span = PARAMS.phase_jitter_span_px
    ax.add_patch(Rectangle((col - span / 2, row - span / 2), span, span, facecolor="none", edgecolor=VERMILLION, lw=1.2))
    ax.text(col, row - 14, f"phase-jitter span\n({span:g} px square)", fontsize=7.5, color=VERMILLION, ha="center", va="bottom")
    ax.set_xlim(-30, width + 330)
    ax.set_ylim(height + 30, -50)
    ax.set_aspect("equal")
    ax.set_xlabel("image column (px), H", fontsize=9)
    ax.set_ylabel("image row (px), V", fontsize=9)
    ax.set_title("(b) field positions in the image (codes 0 to 4)", fontsize=10)
    # Inset: the span magnified, pixel grid drawn.
    inset = ax.inset_axes([0.745, 0.3, 0.24, 0.32])
    half = span / 2.0 + INSET_PIXELS
    inset.set_xlim(-half, half)
    inset.set_ylim(half, -half)
    inset.set_aspect("equal")
    inset.add_patch(Rectangle((-span / 2, -span / 2), span, span, facecolor=VERMILLION, alpha=0.15, edgecolor=VERMILLION, lw=1.4))
    ticks = [t for t in range(-int(half), int(half) + 1)]
    for t in ticks:
        inset.axhline(t, color=GRAY, lw=0.3)
        inset.axvline(t, color=GRAY, lw=0.3)
    inset.plot([0], [0], marker="+", color=BLACK, ms=8)
    inset.set_xticks([])
    inset.set_yticks([])
    ax.add_artist(ConnectionPatch(xyA=(col + span / 2, row), coordsA=ax.transData, xyB=(-half, 0.0),
                                  coordsB=inset.transData, color=VERMILLION, lw=0.8))
    inset.set_title("the span, magnified (grid = pixels)", fontsize=6.5, pad=2)
    return len(positions)


def main() -> None:
    fig, (ax_z, ax_f) = plt.subplots(1, 2, figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI,
                                     gridspec_kw={"width_ratios": [1.15, 1.0]})
    noise, shape, reduced = draw_z_stations(ax_z)
    positions = draw_field_positions(ax_f)
    fig.subplots_adjust(left=0.05, right=0.99, top=0.92, bottom=0.12, wspace=0.12)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, noise_station_count=noise, shape_station_count=shape, field_position_count=positions,
                  reduced_station_count=reduced)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
