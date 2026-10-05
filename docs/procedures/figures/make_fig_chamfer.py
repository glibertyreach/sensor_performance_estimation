"""
fig_chamfer.png -- cross-section of a chamfered (knife-edge) cutout and of a chamfered disk on its
post, NOT TO SCALE, with the front plate, the back plate at the gap G, and the steepest rays of the
left camera, the right camera and the projector passing the knife edge.

What it shows (specification Section 3.3):
    - the bevel is cut from the BACK of the front plate, so a cutout widens away from the sensor
      and a disk narrows away from the sensor (a frustum);
    - a flat land of at most EDGE_LAND_MAX_MM is left at the front edge;
    - the bevel angle, measured from the plate normal, must exceed the worst-case ray angle plus
      the margin CHAMFER_MARGIN_DEG, so that no ray meets the bevel wall: the sensor sees only the
      front face and the back plate.
The drawn angles are illustration values (named below); the real bevel angle is checked against
the real sensor geometry by the engineer (section 2 of the procedure). The margin and the land
limit are read from CharacterizationParameters.

    python3 docs/procedures/figures/make_fig_chamfer.py      (from the repository root)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import figfacts  # noqa: E402
from sensorperf.parameters import CharacterizationParameters  # noqa: E402

PARAMS = CharacterizationParameters()

OUTPUT_DPI = 200
FIGURE_SIZE_IN = (13.0, 5.2)
FIGURE_NAME = "fig_chamfer"

# Okabe-Ito palette.
BLACK = "#000000"
ORANGE = "#E69F00"
SKY_BLUE = "#56B4E9"
BLUISH_GREEN = "#009E73"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
GRAY = "#595959"
LIGHT_GRAY = "#D9D9D9"

# Illustration geometry (drawing units, not millimeters).
HALF_OPENING = 30.0          # half width of the feature at the front face
PLATE_THICKNESS = 14.0
LAND = 2.0                   # drawn land (the real limit is EDGE_LAND_MAX_MM)
GAP = 55.0                   # drawn gap G between front and back plate
BACK_PLATE_THICKNESS = 8.0
BEVEL_DEG = 48.0             # drawn bevel angle from the plate normal
RAY_DEG = 30.0               # drawn worst-case ray angle from the plate normal (left and right cameras)
CAMERA_HEIGHT = 70.0        # height of the rays' origins above the front face
RAY_END_PAD = 4.0
POST_HALF_WIDTH = 2.0
X_LIMITS = (-100.0, 125.0)
Y_TOP = -CAMERA_HEIGHT - 62.0
L_COLOR, R_COLOR, P_COLOR = BLUE, VERMILLION, BLUISH_GREEN


SOURCE_SPACING = 12.0        # drawn spacing of the three viewpoints (they are close together compared with Z)
SOURCES = (("left camera", L_COLOR, 0), ("projector", P_COLOR, 1), ("right camera", R_COLOR, 2))


def path_arc(ax, center, radius, phi_deg, color, direction=+1):
    """An arc from the plate normal (pointing away from the sensor, +z) through phi degrees, drawn as a polyline
    (the z axis points down in this view, so matplotlib's Arc angles would be mirrored)."""
    phis = [math.radians(phi_deg * k / 30.0) for k in range(31)]
    xs = [center[0] + direction * radius * math.sin(a) for a in phis]
    zs = [center[1] + radius * math.cos(a) for a in phis]
    ax.plot(xs, zs, color=color, lw=1.0)


def draw_rays(ax, edge_x: float) -> None:
    """Rays of the three viewpoints that graze the knife edge at (edge_x, 0) coming from the upper left.
    All three sources lie on one side of the feature (a feature far off axis), so the rays are nearly
    parallel; the left camera's is the steepest. Each ray continues to the back plate."""
    slope = math.tan(math.radians(RAY_DEG))
    z_end = PLATE_THICKNESS + GAP + RAY_END_PAD
    base_x = edge_x - slope * CAMERA_HEIGHT                      # left camera position at the top
    for name, color, index in SOURCES:
        src = (base_x + index * SOURCE_SPACING, -CAMERA_HEIGHT)
        dx, dz = edge_x - src[0], CAMERA_HEIGHT
        end = (edge_x + dx / dz * z_end, z_end)
        ax.plot([src[0], end[0]], [src[1], end[1]], color=color, lw=1.6)
        ax.plot([src[0]], [src[1]], marker="o", ms=5, color=color)
        ax.text(src[0] - 4 + 0 * index, src[1] - 7 - 15 * (2 - index), name, fontsize=8, color=color, ha="right"
                if index == 0 else "center", va="bottom")
        ax.plot([src[0], src[0]], [src[1] - 5, src[1] - 7 - 15 * (2 - index)], color=color, lw=0.5)


def draw_back_plate(ax) -> None:
    top = PLATE_THICKNESS + GAP
    ax.add_patch(Rectangle((X_LIMITS[0] + 5, top), X_LIMITS[1] - X_LIMITS[0] - 10, BACK_PLATE_THICKNESS,
                           facecolor=LIGHT_GRAY, edgecolor=BLACK, lw=1.2, hatch="////"))
    ax.text(X_LIMITS[0] + 8, top + BACK_PLATE_THICKNESS + 8, "back plate, same finish", fontsize=8, va="top")
    ax.annotate("", xy=(X_LIMITS[0] + 14, PLATE_THICKNESS), xytext=(X_LIMITS[0] + 14, top),
                arrowprops=dict(arrowstyle="<->", lw=0.9))
    ax.text(X_LIMITS[0] + 18, PLATE_THICKNESS + GAP / 2, "gap G", fontsize=8, ha="left", va="center")


def angle_marks(ax, edge_x: float) -> None:
    """Plate normal through the edge, the worst-case ray angle and the bevel angle (both from the normal,
    both measured on the side of the plate away from the sensor)."""
    ax.plot([edge_x, edge_x], [-40, PLATE_THICKNESS + GAP], color=GRAY, lw=0.8, linestyle=":")
    ax.text(edge_x + 2, -44, "plate normal", fontsize=6.5, color=GRAY, ha="left")
    path_arc(ax, (edge_x, 0.0), 30.0, RAY_DEG, L_COLOR)
    path_arc(ax, (edge_x, 0.0), 41.0, BEVEL_DEG, BLACK)


def draw_cutout(ax) -> None:
    ax.set_title("(a) cutout or window: countersunk from the back", fontsize=9.5)
    tan_b = math.tan(math.radians(BEVEL_DEG))
    back_half = HALF_OPENING + (PLATE_THICKNESS - LAND) * tan_b
    outer_limit = X_LIMITS[1] - 5
    left_limit = -(-X_LIMITS[0] - 5)
    for sign in (-1, 1):
        outer = sign * outer_limit
        pts = [(outer, 0), (sign * HALF_OPENING, 0), (sign * HALF_OPENING, LAND), (sign * back_half, PLATE_THICKNESS),
               (outer, PLATE_THICKNESS)]
        ax.add_patch(Polygon(pts, closed=True, facecolor=SKY_BLUE, alpha=0.55, edgecolor=BLACK, lw=1.2))
    ax.text(-outer_limit + 4, -8, "front plate", fontsize=8, ha="left", va="bottom")
    draw_rays(ax, HALF_OPENING)
    draw_back_plate(ax)
    angle_marks(ax, HALF_OPENING)
    ax.annotate("ray angle", xy=(HALF_OPENING + 30 * math.sin(math.radians(RAY_DEG)),
                                 30 * math.cos(math.radians(RAY_DEG))), xytext=(95, 36), fontsize=7.5, color=L_COLOR,
                arrowprops=dict(arrowstyle="-", color=L_COLOR, lw=0.7), va="center")
    ax.annotate("bevel angle", xy=(HALF_OPENING + 41 * math.sin(math.radians(BEVEL_DEG)),
                                   41 * math.cos(math.radians(BEVEL_DEG))), xytext=(95, 24), fontsize=7.5,
                color=BLACK, arrowprops=dict(arrowstyle="-", lw=0.7), va="center")
    ax.annotate("land at the front edge\n(at most the limit)", xy=(HALF_OPENING + 0.5, LAND / 2), xytext=(62, -22),
                fontsize=7.5, ha="left", va="center", arrowprops=dict(arrowstyle="-", lw=0.7))
    ax.text(0, PLATE_THICKNESS + GAP + BACK_PLATE_THICKNESS + 26,
            "no ray meets the bevel wall", fontsize=8, ha="center", color=GRAY)


def draw_disk(ax) -> None:
    ax.set_title("(b) disk or raised square on a post: bevelled behind", fontsize=9.5)
    tan_b = math.tan(math.radians(BEVEL_DEG))
    back_half = max(HALF_OPENING - (PLATE_THICKNESS - LAND) * tan_b, POST_HALF_WIDTH + 1.0)
    pts = [(-HALF_OPENING, 0), (HALF_OPENING, 0), (HALF_OPENING, LAND), (back_half, PLATE_THICKNESS),
           (-back_half, PLATE_THICKNESS), (-HALF_OPENING, LAND)]
    ax.add_patch(Polygon(pts, closed=True, facecolor=SKY_BLUE, alpha=0.55, edgecolor=BLACK, lw=1.2))
    ax.add_patch(Rectangle((-POST_HALF_WIDTH, PLATE_THICKNESS), 2 * POST_HALF_WIDTH, GAP, facecolor=GRAY,
                           edgecolor=BLACK, lw=0.8))
    ax.text(POST_HALF_WIDTH + 6, PLATE_THICKNESS + GAP * 0.78, "thin post\n(behind the disk)", fontsize=7.5,
            va="center")
    ax.text(HALF_OPENING + 4, -4, "disk front face", fontsize=8, ha="left", va="bottom")
    draw_rays(ax, -HALF_OPENING)
    draw_back_plate(ax)
    angle_marks(ax, -HALF_OPENING)
    ax.annotate("ray angle", xy=(-HALF_OPENING + 30 * math.sin(math.radians(RAY_DEG)),
                                 30 * math.cos(math.radians(RAY_DEG))), xytext=(50, 36), fontsize=7.5, color=L_COLOR,
                arrowprops=dict(arrowstyle="-", color=L_COLOR, lw=0.7), va="center")
    ax.annotate("bevel angle", xy=(-HALF_OPENING + 41 * math.sin(math.radians(BEVEL_DEG)),
                                   41 * math.cos(math.radians(BEVEL_DEG))), xytext=(50, 24), fontsize=7.5,
                color=BLACK, arrowprops=dict(arrowstyle="-", lw=0.7), va="center")
    ax.text(0, PLATE_THICKNESS + GAP + BACK_PLATE_THICKNESS + 26,
            "the sensor sees the front face and the back plate, never the bevel", fontsize=8, ha="center",
            color=GRAY)


def main() -> None:
    fig, axes = plt.subplots(1, 2, figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI)
    for ax, draw in zip(axes, (draw_cutout, draw_disk)):
        draw(ax)
        ax.set_xlim(*X_LIMITS)
        ax.set_ylim(PLATE_THICKNESS + GAP + BACK_PLATE_THICKNESS + 40, Y_TOP)    # z grows downward, away from the sensor
        ax.set_aspect("equal")
        ax.axis("off")
        # Title was drawn by set_title before axis off; re-enable it explicitly.
    for ax in axes:
        ax.title.set_visible(True)
    fig.text(0.5, 0.02,
             f"Not to scale. Rule: bevel angle (from the plate normal) at least the worst-case ray angle of the three "
             f"views at any pose used, plus {PARAMS.chamfer_margin_deg:g} degrees of margin; land at most "
             f"{PARAMS.edge_land_max_mm:g} mm.", ha="center", fontsize=8.5)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.93, bottom=0.07, wspace=0.02)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, panel_count=2, chamfer_margin_deg=PARAMS.chamfer_margin_deg)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
