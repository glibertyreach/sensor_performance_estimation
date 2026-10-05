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
from matplotlib.patches import Arc, Polygon, Rectangle

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
PROJECTOR_RAY_DEG = 13.0     # drawn ray angle of the projector (nearer the axis)
CAMERA_HEIGHT = 135.0        # height of the rays' origins above the front face
RAY_END_PAD = 4.0
POST_HALF_WIDTH = 2.0
X_LIMITS = (-105.0, 105.0)
Y_TOP = -CAMERA_HEIGHT - 28.0
L_COLOR, R_COLOR, P_COLOR = BLUE, VERMILLION, BLUISH_GREEN


def ray(ax, edge_x: float, angle_deg: float, direction: int, color: str, label: str | None, label_dx: float) -> None:
    """A ray that grazes the knife edge at (edge_x, 0). direction = +1 heads toward +x as it descends
    (rays from a source on the left), -1 toward -x. Drawn from CAMERA_HEIGHT above the front face to
    just past the back plate's front surface."""
    slope = math.tan(math.radians(angle_deg))
    z_end = PLATE_THICKNESS + GAP + RAY_END_PAD
    start = (edge_x - direction * slope * CAMERA_HEIGHT, -CAMERA_HEIGHT)
    end = (edge_x + direction * slope * z_end, z_end)
    ax.plot([start[0], end[0]], [start[1], end[1]], color=color, lw=1.6)
    ax.plot([start[0]], [start[1]], marker="o", ms=5, color=color)
    if label:
        ax.text(start[0] + label_dx, start[1] - 6, label, fontsize=8, color=color, ha="center", va="bottom")


def angle_marks(ax, edge_x: float, bevel_dir: int, ray_dir: int, title_y: float) -> None:
    """Dotted plate normal at the edge, the arc of the worst-case ray angle and the arc of the bevel angle."""
    ax.plot([edge_x, edge_x], [-62, 62], color=GRAY, lw=0.8, linestyle=":")
    ax.text(edge_x + 2, -64, "plate normal", fontsize=6.5, color=GRAY, ha="left")


def draw_back_plate(ax) -> None:
    top = PLATE_THICKNESS + GAP
    ax.add_patch(Rectangle((X_LIMITS[0] + 5, top), X_LIMITS[1] - X_LIMITS[0] - 10, BACK_PLATE_THICKNESS,
                           facecolor=LIGHT_GRAY, edgecolor=BLACK, lw=1.2, hatch="////"))
    ax.text(X_LIMITS[0] + 8, top + BACK_PLATE_THICKNESS + 10, "back plate, same finish", fontsize=8, va="top")
    ax.annotate("", xy=(X_LIMITS[1] - 12, PLATE_THICKNESS), xytext=(X_LIMITS[1] - 12, top),
                arrowprops=dict(arrowstyle="<->", lw=0.9))
    ax.text(X_LIMITS[1] - 16, PLATE_THICKNESS + GAP / 2, "gap G", fontsize=8, ha="right", va="center")


def draw_cutout(ax) -> None:
    ax.set_title("(a) cutout or window: countersunk from the back", fontsize=9.5)
    tan_b = math.tan(math.radians(BEVEL_DEG))
    back_half = HALF_OPENING + (PLATE_THICKNESS - LAND) * tan_b
    left_x, right_x = X_LIMITS
    # The front plate, with the opening: polygon traced around the solid on each side.
    for sign in (-1, 1):
        outer = sign * (right_x - 5)
        pts = [(outer, 0), (sign * HALF_OPENING, 0), (sign * HALF_OPENING, LAND), (sign * back_half, PLATE_THICKNESS),
               (outer, PLATE_THICKNESS)]
        ax.add_patch(Polygon(pts, closed=True, facecolor=SKY_BLUE, alpha=0.55, edgecolor=BLACK, lw=1.2))
    ax.text(-right_x + 12, -9, "front plate (front face)", fontsize=8, ha="left", va="bottom")
    # Rays: left camera past the right edge, right camera past the left edge, projector past both.
    ray(ax, HALF_OPENING, RAY_DEG, +1, L_COLOR, "left camera", 0.0)
    ray(ax, -HALF_OPENING, RAY_DEG, -1, R_COLOR, "right camera", 0.0)
    ray(ax, HALF_OPENING, PROJECTOR_RAY_DEG, -1, P_COLOR, None, 0.0)
    ray(ax, -HALF_OPENING, PROJECTOR_RAY_DEG, +1, P_COLOR, "projector", 0.0)
    draw_back_plate(ax)
    # Angle marks at the right edge: ray angle and bevel angle from the plate normal.
    edge = (HALF_OPENING, 0.0)
    ax.plot([edge[0], edge[0]], [-35, 50], color=GRAY, lw=0.8, linestyle=":")
    ax.add_patch(Arc(edge, 46, 46, theta1=90 - RAY_DEG, theta2=90, angle=0, color=L_COLOR, lw=1.0, ))
    ax.add_patch(Arc(edge, 62, 62, theta1=90 - BEVEL_DEG, theta2=90, angle=0, color=BLACK, lw=1.0))
    ax.text(edge[0] + 16, 28, "worst-case\nray angle", fontsize=7.5, color=L_COLOR, ha="left", va="top")
    ax.text(edge[0] + 36, 56, "bevel angle from the plate normal", fontsize=7.5, color=BLACK, ha="left", va="center")
    # Land label.
    ax.annotate("land at the front edge\n(at most the limit)", xy=(HALF_OPENING - 1, LAND / 2), xytext=(-18, -34),
                fontsize=7.5, ha="center", arrowprops=dict(arrowstyle="-", lw=0.7))
    ax.annotate("knife edge", xy=(-HALF_OPENING, 0), xytext=(-72, -36), fontsize=7.5, ha="center",
                arrowprops=dict(arrowstyle="-", lw=0.7))
    ax.text(0, PLATE_THICKNESS + GAP * 0.55, "back plate seen through the opening", fontsize=7.5, ha="center", color=GRAY)
    ax.text(0, PLATE_THICKNESS + 12, "open space: no ray meets the bevel wall", fontsize=7.5, ha="center", color=GRAY)


def draw_disk(ax) -> None:
    ax.set_title("(b) disk or raised square on a post: bevelled behind", fontsize=9.5)
    tan_b = math.tan(math.radians(BEVEL_DEG))
    back_half = max(HALF_OPENING - (PLATE_THICKNESS - LAND) * tan_b, POST_HALF_WIDTH + 1.0)
    pts = [(-HALF_OPENING, 0), (HALF_OPENING, 0), (HALF_OPENING, LAND), (back_half, PLATE_THICKNESS),
           (-back_half, PLATE_THICKNESS), (-HALF_OPENING, LAND)]
    ax.add_patch(Polygon(pts, closed=True, facecolor=SKY_BLUE, alpha=0.55, edgecolor=BLACK, lw=1.2))
    ax.add_patch(Rectangle((-POST_HALF_WIDTH, PLATE_THICKNESS), 2 * POST_HALF_WIDTH, GAP, facecolor=GRAY,
                           edgecolor=BLACK, lw=0.8))
    ax.text(POST_HALF_WIDTH + 4, PLATE_THICKNESS + GAP * 0.45, "thin post\n(behind the disk)", fontsize=7.5, va="center")
    ax.text(0, -9, "disk front face", fontsize=8, ha="center", va="bottom")
    # Rays: steepest at the edge on the far side of each source.
    ray(ax, -HALF_OPENING, RAY_DEG, +1, L_COLOR, "left camera", 0.0)
    ray(ax, HALF_OPENING, RAY_DEG, -1, R_COLOR, "right camera", 0.0)
    ray(ax, -HALF_OPENING, PROJECTOR_RAY_DEG, -1, P_COLOR, None, 0.0)
    ray(ax, HALF_OPENING, PROJECTOR_RAY_DEG, +1, P_COLOR, "projector", 0.0)
    draw_back_plate(ax)
    edge = (-HALF_OPENING, 0.0)
    ax.plot([edge[0], edge[0]], [-35, 50], color=GRAY, lw=0.8, linestyle=":")
    ax.add_patch(Arc(edge, 46, 46, theta1=90, theta2=90 + RAY_DEG, angle=0, color=L_COLOR, lw=1.0))
    ax.add_patch(Arc(edge, 62, 62, theta1=90, theta2=90 + BEVEL_DEG, angle=0, color=BLACK, lw=1.0))
    ax.text(edge[0] - 16, 28, "worst-case\nray angle", fontsize=7.5, color=L_COLOR, ha="right", va="top")
    ax.text(edge[0] - 36, 56, "bevel angle", fontsize=7.5, color=BLACK, ha="right", va="center")
    ax.text(0, PLATE_THICKNESS + GAP + BACK_PLATE_THICKNESS + 28,
            "the sensor sees the disk front face and the back plate, never the bevel", fontsize=7.5, ha="center",
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
