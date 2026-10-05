"""
fig_targets.png -- front views, to one common scale, of the seven targets the technician mounts
for the edge, noise, area and detection series: T2, T3a, T3b, T4 and T5.

The layouts are not drawn by hand. They come from make_standard_target_set(), called with the
default CharacterizationParameters and the indicative SensorGeometry, so the plate sizes, the
feature ladder, the blank sites, the post sites and the 5 degree slant are exactly those the
analysis code will read from targets.json. The real layout is fixed by the fabricator from the
final sensor geometry (the f_x value is unconfirmed); this figure shows the structure.

Legend:
    solid blue          front surface (the nearer plane); a disk or the raised square is front material
    gray                the back plate, seen around a disk or through a cutout (at the gap G behind)
    dashed circle       blank site (no feature; its size is the search window it serves)
    vermillion dot      post-only control site (a post with no disk)
Each panel carries a 100 mm scale bar. The smallest features are far below one drawing pixel; they
are drawn at true size with a hairline edge so that their place is still visible.

    python3 docs/procedures/figures/make_fig_targets.py      (from the repository root)
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import figfacts  # noqa: E402
from sensorperf.geometry.targets import (  # noqa: E402
    FEATURE_BLANK, FEATURE_CUTOUT, FEATURE_DISK, FEATURE_POST, FEATURE_SQUARE_RAISED, FEATURE_SQUARE_WINDOW,
    make_standard_target_set,
)
from sensorperf.parameters import (  # noqa: E402
    CharacterizationParameters, SensorGeometry, TARGET_CUTOUTS, TARGET_DISKS, TARGET_NOISE_PLATE,
    TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW,
)

PARAMS = CharacterizationParameters()
GEOMETRY = SensorGeometry.indicative()

OUTPUT_DPI = 200
FIGURE_WIDTH_IN = 13.0
FIGURE_NAME = "fig_targets"
SCALE_BAR_MM = 100.0
PANEL_PAD_MM = 22.0
ROW_GAP_MM = 30.0
MARGIN_FRACTION = 0.02
TITLE_HEIGHT_IN = 0.6
CAPTION_HEIGHT_IN = 0.45

# Okabe-Ito palette.
BLACK = "#000000"
SKY_BLUE = "#56B4E9"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
GRAY = "#595959"
BACK_GRAY = "#D9D9D9"
FRONT_ALPHA = 0.55

ROW_ONE = (TARGET_NOISE_PLATE, TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW)
ROW_TWO = (TARGET_DISKS, TARGET_CUTOUTS)
TITLES = {
    TARGET_NOISE_PLATE: "T2 noise and registration plate",
    TARGET_RAISED_SQUARE: "T3a raised square",
    TARGET_SQUARE_WINDOW: "T3b square window",
    TARGET_DISKS: "T4 disk plate",
    TARGET_CUTOUTS: "T5 cutout plate",
}


def plate_rectangle(target, back: bool) -> Rectangle:
    half_w, half_h = target.back_extent() if back else (target.half_width_mm, target.half_height_mm)
    return Rectangle((-half_w, -half_h), 2 * half_w, 2 * half_h)


def draw_target(ax, target) -> None:
    """One front view. Image rows grow downward, so the y axis is inverted (V down)."""
    half_w, half_h = target.half_width_mm, target.half_height_mm
    kind_has_back = target.gap_mm is not None
    # Back plate (gray) behind the front surface where there is one.
    if kind_has_back:
        back = plate_rectangle(target, back=True)
        back.set(facecolor=BACK_GRAY, edgecolor=GRAY, lw=0.8)
        ax.add_patch(back)
    squares = [f for f in target.features if f.kind in (FEATURE_SQUARE_RAISED, FEATURE_SQUARE_WINDOW)]
    if target.kind == "plate":
        ax.add_patch(Rectangle((-half_w, -half_h), 2 * half_w, 2 * half_h, facecolor=SKY_BLUE, alpha=FRONT_ALPHA,
                               edgecolor=BLACK, lw=1.0))
    elif target.kind == "raised_square":
        for f in squares:
            ax.add_patch(Polygon(f.outline_points(), closed=True, facecolor=SKY_BLUE, alpha=FRONT_ALPHA + 0.15,
                                 edgecolor=BLUE, lw=1.2))
    elif target.kind == "square_window":
        ax.add_patch(Rectangle((-half_w, -half_h), 2 * half_w, 2 * half_h, facecolor=SKY_BLUE, alpha=FRONT_ALPHA,
                               edgecolor=BLACK, lw=1.0))
        for f in squares:
            ax.add_patch(Polygon(f.outline_points(), closed=True, facecolor=BACK_GRAY, edgecolor=BLUE, lw=1.2))
    elif target.kind == "cutout_array":
        ax.add_patch(Rectangle((-half_w, -half_h), 2 * half_w, 2 * half_h, facecolor=SKY_BLUE, alpha=FRONT_ALPHA,
                               edgecolor=BLACK, lw=1.0))
    for f in target.features:
        if f.kind == FEATURE_DISK:
            ax.add_patch(Circle((f.x_mm, f.y_mm), f.diameter_mm / 2, facecolor=BLUE, edgecolor=BLUE, lw=0.5))
        elif f.kind == FEATURE_CUTOUT:
            ax.add_patch(Circle((f.x_mm, f.y_mm), f.diameter_mm / 2, facecolor=BACK_GRAY, edgecolor=BLUE, lw=0.5))
        elif f.kind == FEATURE_BLANK:
            ax.add_patch(Circle((f.x_mm, f.y_mm), f.diameter_mm / 2, facecolor="none", edgecolor=BLACK, lw=0.6,
                                linestyle=(0, (3, 2))))
        elif f.kind == FEATURE_POST:
            ax.plot([f.x_mm], [f.y_mm], marker="o", ms=3.2, color=VERMILLION, zorder=6)
    # Scale bar, bottom left inside the panel margin.
    y_bar = half_h + PANEL_PAD_MM * 0.55
    ax.plot([-half_w, -half_w + SCALE_BAR_MM], [y_bar, y_bar], color=BLACK, lw=2.0, solid_capstyle="butt")
    ax.text(-half_w + SCALE_BAR_MM + 6, y_bar, f"{SCALE_BAR_MM:g} mm", fontsize=7, va="center")
    ax.set_xlim(-half_w - PANEL_PAD_MM, half_w + PANEL_PAD_MM)
    ax.set_ylim(half_h + PANEL_PAD_MM, -half_h - PANEL_PAD_MM)
    ax.set_aspect("equal")
    ax.axis("off")


def describe(target) -> str:
    width, height = 2 * target.half_width_mm, 2 * target.half_height_mm
    text = f"{TITLES[target.target_id]}\n{width:.0f} x {height:.0f} mm"
    if target.target_id in (TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW):
        square = target.features[0]
        text += f"\nsquare {square.diameter_mm:g} mm, slant {square.rotation_deg:g} deg"
    elif target.features:
        sizes = [f.diameter_mm for f in target.features if f.kind in (FEATURE_DISK, FEATURE_CUTOUT)]
        blanks = [f for f in target.features if f.kind == FEATURE_BLANK]
        posts = [f for f in target.features if f.kind == FEATURE_POST]
        text += f"\n{len(sizes)} features: {figfacts.feature_diameters_text(sorted(sizes))}\n{len(blanks)} blank sites"
        if posts:
            text += f", {len(posts)} post-only site"
    return text


def main() -> None:
    targets = make_standard_target_set(PARAMS, GEOMETRY)
    diameters = PARAMS.feature_diameters_mm(GEOMETRY)

    def panel_size_mm(target_id):
        target = targets.targets[target_id]
        return 2 * target.half_width_mm + 2 * PANEL_PAD_MM, 2 * target.half_height_mm + 2 * PANEL_PAD_MM

    # One common scale (inches per mm): the widest row fills the figure width.
    rows = (ROW_ONE, ROW_TWO)
    row_width_mm = [sum(panel_size_mm(i)[0] for i in ids) + ROW_GAP_MM * (len(ids) - 1) for ids in rows]
    row_height_mm = [max(panel_size_mm(i)[1] for i in ids) for ids in rows]
    inch_per_mm = FIGURE_WIDTH_IN * (1.0 - 2 * MARGIN_FRACTION) / max(row_width_mm)
    title_in, caption_in = TITLE_HEIGHT_IN, CAPTION_HEIGHT_IN
    figure_height = sum(row_height_mm) * inch_per_mm + 2 * title_in + caption_in
    fig = plt.figure(figsize=(FIGURE_WIDTH_IN, figure_height), dpi=OUTPUT_DPI)
    y_top = figure_height
    for ids, height_mm in zip(rows, row_height_mm):
        y_top -= title_in
        y_bottom = y_top - height_mm * inch_per_mm
        x = MARGIN_FRACTION * FIGURE_WIDTH_IN
        for target_id in ids:
            width_mm, panel_height_mm = panel_size_mm(target_id)
            ax = fig.add_axes([x / FIGURE_WIDTH_IN, (y_top - panel_height_mm * inch_per_mm) / figure_height,
                               width_mm * inch_per_mm / FIGURE_WIDTH_IN, panel_height_mm * inch_per_mm / figure_height])
            target = targets.targets[target_id]
            draw_target(ax, target)
            ax.set_title(describe(target), fontsize=7.5, pad=3)
            x += (width_mm + ROW_GAP_MM) * inch_per_mm
        y_top = y_bottom
    fig.text(0.02, 0.15 / figure_height, "Gray: back plate at the gap G behind the front surface (G is GAP_SMALL_MM or GAP_LARGE_MM, set "
             "with spacers). Dashed circle: blank site. Vermillion dot: post-only control site. Same scale in every panel.",
             fontsize=7.5, color=GRAY)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    blank_counts = {len([f for f in targets.targets[i].features if f.kind == FEATURE_BLANK]) for i in ROW_TWO}
    assert blank_counts == {PARAMS.blank_sites_per_plate}, blank_counts
    figfacts.emit(FIGURE_NAME, feature_count=len(diameters), feature_diameters=figfacts.feature_diameters_text(diameters),
                  blank_site_count=PARAMS.blank_sites_per_plate, target_panel_count=len(ROW_ONE) + len(ROW_TWO))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
