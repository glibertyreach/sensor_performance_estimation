"""
fig_setup.png -- the experimental setup, side view, drawn with the working-volume dimensions of
the code (Z_MIN_MM, Z_MAX_MM, the indicative field of view) and a stylized robot.

What to see:
    - the sensor sits on its own rigid stand at the left; the robot stands on a separate base at
      the right, so robot motion cannot move the sensor (equipment table, section 1);
    - the robot carries a target on the dowel-pinned quick-change adapter, here T2 at the
      reference station, with ghost outlines at Z_MIN and Z_MAX;
    - Z runs along the left IR camera's optical axis from its optical center; V is vertical in
      this view and H points into the page;
    - the frustum is the indicative field of view of the left camera;
    - the dial indicator stands on the floor on its own stand and touches the back of the adapter
      along the target normal (the B-Z series, section 8);
    - the enclosure (or blackout curtains) surrounds everything.
The robot arm is a sketch and is not to scale; the Z axis, the target and the frustum are.

    python3 docs/procedures/figures/make_fig_setup.py      (from the repository root)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import figfacts  # noqa: E402
from sensorperf.parameters import CharacterizationParameters, SensorGeometry  # noqa: E402

PARAMS = CharacterizationParameters()
GEOMETRY = SensorGeometry.indicative()

OUTPUT_DPI = 200
FIGURE_SIZE_IN = (13.0, 5.0)
FIGURE_NAME = "fig_setup"

# Okabe-Ito palette.
BLACK = "#000000"
ORANGE = "#E69F00"
SKY_BLUE = "#56B4E9"
BLUISH_GREEN = "#009E73"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
REDDISH_PURPLE = "#CC79A7"
GRAY = "#595959"
LIGHT_GRAY = "#D9D9D9"

FLOOR_Y_MM = -470.0
"""Floor height relative to the camera axis (a sketch value)."""
SENSOR_BODY_MM = (70.0, 50.0)           # depth, height of the sensor housing (sketch)
SENSOR_STAND_WIDTH_MM = 90.0
TARGET_THICKNESS_MM = 14.0
ADAPTER_THICKNESS_MM = 30.0
ADAPTER_HALF_HEIGHT_MM = 110.0
FLANGE_THICKNESS_MM = 16.0
FLANGE_HALF_HEIGHT_MM = 45.0
ROBOT_BASE_X_MM = 1560.0
ROBOT_BASE_SIZE_MM = (170.0, 70.0)
ROBOT_ELBOW_MM = (1380.0, 300.0)
ROBOT_SHOULDER_MM = (1560.0, -250.0)
LINK_WIDTH_PT = 14
DIAL_HEIGHT_MM = -200.0                 # height of the indicator tip (below the flange, on the adapter back)
DIAL_STAND_X_MM = 1100.0
ENCLOSURE_MARGIN_MM = (170.0, 40.0)


def target_x(z_mm: float) -> float:
    """x position (= Z) of the target's front face."""
    return z_mm


def draw_target(ax, z_mm: float, half_height_mm: float, ghost: bool) -> None:
    """The target plate (front face at Z) as a thin slab; a ghost is dashed and unfilled."""
    style = dict(edgecolor=GRAY if ghost else BLACK, lw=1.0 if ghost else 1.4,
                 linestyle="--" if ghost else "-", facecolor="none" if ghost else SKY_BLUE, alpha=1.0)
    ax.add_patch(Rectangle((z_mm, -half_height_mm), TARGET_THICKNESS_MM, 2 * half_height_mm, **style))


def main() -> None:
    z_min, z_max, z_ref = PARAMS.z_min_mm, PARAMS.z_max_mm, PARAMS.z_reference_mm
    half_plate = PARAMS.noise_plate_size_mm[1] / 2.0
    half_fov_v = math.atan(GEOMETRY.image_height_px / 2.0 / GEOMETRY.sensor_fy_px)
    frustum_depth = z_max + 140.0

    fig, ax = plt.subplots(figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI)

    # Frustum of the left camera (vertical half field of view of the indicative geometry).
    reach = frustum_depth
    ax.add_patch(Polygon([(0, 0), (reach, reach * math.tan(half_fov_v)), (reach, -reach * math.tan(half_fov_v))],
                         closed=True, facecolor=BLUISH_GREEN, alpha=0.10, edgecolor=BLUISH_GREEN, lw=1.0,
                         linestyle="-"))
    ax.text(430.0, -300.0, "frustum of the left IR camera\n(indicative field of view)", fontsize=7.5,
            color=BLUISH_GREEN, ha="left", va="top")

    # Floor and the two mechanically separate foundations.
    gap_left, gap_right = 360.0, 880.0
    ax.plot([-250, gap_left], [FLOOR_Y_MM, FLOOR_Y_MM], color=BLACK, lw=2)
    ax.plot([gap_right, 1800], [FLOOR_Y_MM, FLOOR_Y_MM], color=BLACK, lw=2)
    ax.text((gap_left + gap_right) / 2, FLOOR_Y_MM - 30, "separate foundations", ha="center", va="top", fontsize=7.5,
            color=GRAY)
    ax.add_patch(Rectangle((-250, FLOOR_Y_MM - 40), gap_left + 250, 40, facecolor=LIGHT_GRAY, edgecolor=BLACK,
                           hatch="////", lw=0.8))
    ax.add_patch(Rectangle((gap_right, FLOOR_Y_MM - 40), 1800 - gap_right, 40, facecolor=LIGHT_GRAY, edgecolor=BLACK,
                           hatch="////", lw=0.8))

    # Sensor, its rigid stand, and the Z axis.
    body_w, body_h = SENSOR_BODY_MM
    ax.add_patch(Rectangle((-body_w, -body_h / 2), body_w, body_h, facecolor=BLUE, edgecolor=BLACK, lw=1.2))
    ax.add_patch(Rectangle((-body_w / 2 - SENSOR_STAND_WIDTH_MM / 2, FLOOR_Y_MM), SENSOR_STAND_WIDTH_MM,
                           -FLOOR_Y_MM - body_h / 2, facecolor=LIGHT_GRAY, edgecolor=BLACK, lw=1.2))
    ax.text(-body_w / 2, body_h / 2 + 20, "VSX3000 sensor", ha="center", fontsize=8, color=BLUE)
    ax.text(40.0, -250.0, "rigid stand,\nseparate from\nthe robot", ha="left", va="center",
            fontsize=7.5, color=BLACK)
    ax.annotate("", xy=(z_max + 190, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", lw=1.2, color=BLACK))
    ax.text(z_max + 200, 8, "Z", fontsize=9, va="bottom")
    ax.annotate("", xy=(0, 150), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", lw=1.2, color=BLACK))
    ax.text(10, 150, "V (H points into the page)", fontsize=7.5, va="bottom")
    ax.plot([0], [0], marker="o", color=BLACK, ms=3)
    ax.text(40, -26, "origin: optical center\nof the left IR camera", fontsize=7, va="top")

    # Working range Z_MIN .. Z_MAX, drawn as a dimension line above the target.
    y_dim = 420.0
    ax.annotate("", xy=(z_max, y_dim), xytext=(z_min, y_dim), arrowprops=dict(arrowstyle="<->", lw=1.2, color=VERMILLION))
    for z, name in ((z_min, "Z_MIN"), (z_max, "Z_MAX")):
        ax.plot([z, z], [y_dim - 20, y_dim + 20], color=VERMILLION, lw=1.2)
        ax.text(z, y_dim + 28, f"{name} = {z:g} mm", ha="center", fontsize=8, color=VERMILLION)
    ax.text((z_min + z_max) / 2, y_dim - 14, "working range of the test", ha="center", va="top", fontsize=8,
            color=VERMILLION)

    # Targets: ghosts at Z_MIN and Z_MAX, solid at the reference station on the adapter.
    for z in (z_min, z_max):
        draw_target(ax, z, half_plate, ghost=True)
    draw_target(ax, z_ref, half_plate, ghost=False)
    ax.text(z_ref - 20, half_plate - 10, f"target (T2 shown)\nat Z = {z_ref:g} mm", ha="right",
            fontsize=7.5)

    # Dowel-pinned adapter, flange and a stylized arm.
    ax_x = z_ref + TARGET_THICKNESS_MM
    ax.add_patch(Rectangle((ax_x, -ADAPTER_HALF_HEIGHT_MM), ADAPTER_THICKNESS_MM, 2 * ADAPTER_HALF_HEIGHT_MM,
                           facecolor=ORANGE, alpha=0.6, edgecolor=BLACK, lw=1.2))
    ax.plot([ax_x - 6, ax_x + 16], [60, 60], color=BLACK, lw=2.5)     # a dowel pin
    ax.plot([ax_x - 6, ax_x + 16], [-60, -60], color=BLACK, lw=2.5)
    flange_x = ax_x + ADAPTER_THICKNESS_MM
    ax.add_patch(Rectangle((flange_x, -FLANGE_HALF_HEIGHT_MM), FLANGE_THICKNESS_MM, 2 * FLANGE_HALF_HEIGHT_MM,
                           facecolor=GRAY, edgecolor=BLACK, lw=1.2))
    wrist = (flange_x + FLANGE_THICKNESS_MM + 60.0, 0.0)
    ax.plot([flange_x + FLANGE_THICKNESS_MM, wrist[0]], [0, 0], color=GRAY, lw=LINK_WIDTH_PT, solid_capstyle="butt")
    ax.plot([wrist[0], ROBOT_ELBOW_MM[0]], [wrist[1], ROBOT_ELBOW_MM[1]], color=GRAY, lw=LINK_WIDTH_PT,
            solid_capstyle="round")
    ax.plot([ROBOT_ELBOW_MM[0], ROBOT_SHOULDER_MM[0]], [ROBOT_ELBOW_MM[1], ROBOT_SHOULDER_MM[1]], color=GRAY,
            lw=LINK_WIDTH_PT + 2, solid_capstyle="round")
    ax.plot([ROBOT_SHOULDER_MM[0], ROBOT_SHOULDER_MM[0]], [ROBOT_SHOULDER_MM[1], FLOOR_Y_MM], color=GRAY,
            lw=LINK_WIDTH_PT + 6, solid_capstyle="butt")
    for joint in (ROBOT_ELBOW_MM, ROBOT_SHOULDER_MM, wrist):
        ax.add_patch(Circle(joint, 26, facecolor="white", edgecolor=BLACK, lw=1.2, zorder=5))
    ax.add_patch(Rectangle((ROBOT_BASE_X_MM - ROBOT_BASE_SIZE_MM[0] / 2, FLOOR_Y_MM), *ROBOT_BASE_SIZE_MM,
                           facecolor=GRAY, edgecolor=BLACK, lw=1.2))
    ax.text(ROBOT_BASE_X_MM + 130, ROBOT_ELBOW_MM[1] - 40, "6-axis robot\n(sketch, not to scale)", fontsize=7.5,
            ha="left", va="center")
    ax.annotate("dowel-pinned quick-change\nadapter", xy=(ax_x + ADAPTER_THICKNESS_MM / 2, ADAPTER_HALF_HEIGHT_MM),
                xytext=(ax_x + 120, 300), fontsize=7.5, ha="left", color=BLACK,
                arrowprops=dict(arrowstyle="-", color=BLACK, lw=0.8))
    ax.text(flange_x + 30, -FLANGE_HALF_HEIGHT_MM - 8, "robot flange", fontsize=7.5, va="top", ha="left")

    # Dial indicator on its own floor stand, touching the back of the adapter along the target normal.
    stand_w = 26.0
    ax.add_patch(Rectangle((DIAL_STAND_X_MM, FLOOR_Y_MM), stand_w, DIAL_HEIGHT_MM - FLOOR_Y_MM + 20,
                           facecolor=LIGHT_GRAY, edgecolor=BLACK, lw=1.2))
    ax.add_patch(Rectangle((ax_x + ADAPTER_THICKNESS_MM + 2, DIAL_HEIGHT_MM - 10), DIAL_STAND_X_MM - ax_x -
                           ADAPTER_THICKNESS_MM, 20, facecolor=BLUE, alpha=0.5, edgecolor=BLACK, lw=1.0))
    ax.add_patch(Circle((DIAL_STAND_X_MM - 60, DIAL_HEIGHT_MM - 50), 36, facecolor="white", edgecolor=BLUE, lw=1.6))
    ax.annotate("dial indicator on a\nfixed stand (series Z): reads\nthe Z move of the adapter,\nalong the target normal",
                xy=(DIAL_STAND_X_MM - 40, DIAL_HEIGHT_MM - 55), xytext=(DIAL_STAND_X_MM + 50, -350.0),
                fontsize=7.5, color=BLUE, ha="left", va="center", arrowprops=dict(arrowstyle="-", color=BLUE, lw=0.8))

    # Enclosure boundary.
    margin_x, margin_y = ENCLOSURE_MARGIN_MM
    left, right = -body_w - 160.0, 1800.0 + 120.0
    bottom, top = FLOOR_Y_MM - 110.0, y_dim + 110.0
    ax.add_patch(Rectangle((left, bottom), right - left, top - bottom, facecolor="none", edgecolor=VERMILLION,
                           linestyle=(0, (6, 4)), lw=1.3))
    ax.text(left + 15, top - 15, "enclosure or blackout curtains; ambient IR and air temperature logged "
            "(IR light meter, temperature loggers)", fontsize=7.5, color=VERMILLION, va="top")

    ax.set_xlim(left - 40, right + 40)
    ax.set_ylim(bottom - 40, top + 30)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.subplots_adjust(left=0.005, right=0.995, top=0.995, bottom=0.005)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, z_min_mm=z_min, z_max_mm=z_max)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
