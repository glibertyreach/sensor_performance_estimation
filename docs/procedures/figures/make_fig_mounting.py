"""
fig_mounting.png -- the target mounting stack, cross-section through the common axis, not to scale
but proportioned (every thickness and diameter below is drawn at the same scale in both directions).

What to see (section 1b):
    - the robot flange (ISO 9409-1-50-4-M6) at the top carries the target adapter PT-01, which stays on the
      flange;
    - each feature target carries its own spigot PT-02, bolted to the back of its back plate. The spigot drops
      into the 40 mm bore of the adapter with its radial dowel in the keyway, so the spigot axis and the dowel
      together are the datum of every target (the dowel datum, section 2);
    - a ball-lock pin through the adapter's cross hole and the spigot's cross hole holds the spigot in the bore;
    - the standoffs PT-03 hold the front plate at the gap G in front of the back plate. The gap shown is the
      small gap GAP_SMALL_MM; the large gap GAP_LARGE_MM uses longer standoffs.
The axis of the stack runs up the page, robot at the top and sensor side at the bottom. The plates are 300 mm or
more across and are drawn cut off with wavy break lines. The parts are numbered on the drawing and named, with their
drawing numbers, in the key at the right.

    python3 docs/procedures/figures/make_fig_mounting.py      (from the repository root)
"""
from __future__ import annotations

import math
import sys
import textwrap
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import figfacts  # noqa: E402
from sensorperf.parameters import CharacterizationParameters  # noqa: E402

PARAMS = CharacterizationParameters()

OUTPUT_DPI = 200
FIGURE_SIZE_IN = (13.0, 6.0)
FIGURE_NAME = "fig_mounting"

# Okabe-Ito palette (the same as make_fig_setup.py).
BLACK = "#000000"
ORANGE = "#E69F00"
SKY_BLUE = "#56B4E9"
BLUISH_GREEN = "#009E73"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
REDDISH_PURPLE = "#CC79A7"
GRAY = "#595959"
LIGHT_GRAY = "#D9D9D9"

# ---------------------------------------------------------------------------
# Dimensions of the parts, in mm (the drawing numbers of appendix F). The gap comes from the parameters.
# ---------------------------------------------------------------------------
FLANGE_DIAMETER_MM = 63.0                 # ISO 9409-1-50-4-M6
FLANGE_THICKNESS_MM = 14.0                # drawn thickness (the robot's own flange is a sketch)
ADAPTER_WIDTH_MM = 120.0                  # PT-01, aluminum
ADAPTER_THICKNESS_MM = 30.0
BORE_DIAMETER_MM = 40.0
BORE_DEPTH_MM = 25.0
KEYWAY_WIDTH_MM = 6.0
KEYWAY_DEPTH_MM = 3.0                     # radial depth beyond the bore wall (sketch value; the drawing PT-01 governs)
CROSS_HOLE_DIAMETER_MM = 8.0              # through the bore, perpendicular to the axis
CROSS_HOLE_HEIGHT_MM = 15.0               # of the cross-hole axis above the adapter's target-side face
SPIGOT_DIAMETER_MM = 40.0                 # PT-02, steel
SPIGOT_LENGTH_MM = 24.0
SPIGOT_FLANGE_DIAMETER_MM = 80.0
SPIGOT_FLANGE_THICKNESS_MM = 8.0
DOWEL_DIAMETER_MM = 6.0                   # radial dowel; it enters the keyway
DOWEL_HEIGHT_MM = 6.0                     # of the dowel axis above the spigot flange face
BACK_PLATE_THICKNESS_MM = 8.0
FRONT_PLATE_THICKNESS_MM = 6.0
STANDOFF_DIAMETER_MM = 10.0               # PT-03
GAP_SHOWN_MM = PARAMS.gap_small_mm        # the standoff length drawn
GAP_OTHER_MM = PARAMS.gap_large_mm

# Layout of the sketch.
PLATE_HALF_WIDTH_DRAWN_MM = 80.0          # the plates are cut off with break lines at this half width
STANDOFF_RADIUS_MM = 62.0                 # radial position of the two standoffs drawn
BREAK_WAVE_AMPLITUDE_MM = 1.6             # the wavy break line at the end of a cut-off plate
BREAK_STEP_MM = 0.4                       # sampling of the wavy line
PIN_END_BEYOND_ADAPTER_MM = 14.0          # how far the pin's handle end sticks out of the adapter
PIN_HANDLE_SIZE_MM = (7.0, 16.0)          # axial width, radial length of the T-handle
CALLOUT_X_MM = 100.0                      # column of the part numbers, right of the drawing
CALLOUT_RADIUS_MM = 4.6
CALLOUT_SPACING_MM = 10.0                 # least distance between two numbers in the column (two radii plus air)
DOWEL_CALLOUT_X_MM = -80.0                # the dowel's number sits at the left, on its own side of the stack
KEY_X_MM = 118.0                          # left edge of the key (the list of the numbered parts)
KEY_WRAP_CHARACTERS = 52
KEY_LINE_HEIGHT_MM = 4.3
KEY_ITEM_GAP_MM = 3.0
LABEL_FONT_SIZE = 8.5
SMALL_FONT_SIZE = 7.5
LEADER_LW = 0.8
X_LIMITS_MM = (-150.0, 255.0)
CENTERLINE_DASHES = (0, (8, 3, 1.5, 3))


def broken_slab(ax, bottom: float, top: float, half_width: float, **style) -> None:
    """A plate seen edge-on, cut off at both ends with a wavy break line (one wave over the plate's thickness)."""
    steps = max(int(round((top - bottom) / BREAK_STEP_MM)), 2)
    ys = [bottom + (top - bottom) * i / steps for i in range(steps + 1)]
    wave = [BREAK_WAVE_AMPLITUDE_MM * math.sin(2.0 * math.pi * (y - bottom) / (top - bottom)) for y in ys]
    right = [(half_width + w, y) for w, y in zip(wave, ys)]
    left = [(-half_width + w, y) for w, y in zip(wave, ys)]
    ax.add_patch(Polygon(right + left[::-1], closed=True, **style))


def callout(ax, number: int, center_xy, target_xy, color: str = BLACK) -> None:
    """A numbered circle at center_xy with a thin leader line to the part at target_xy."""
    ax.plot([center_xy[0], target_xy[0]], [center_xy[1], target_xy[1]], color=color, lw=LEADER_LW, zorder=7)
    ax.plot([target_xy[0]], [target_xy[1]], marker="o", color=color, ms=2.5, zorder=8)
    ax.add_patch(Circle(center_xy, CALLOUT_RADIUS_MM, facecolor="white", edgecolor=color, lw=1.1, zorder=9))
    ax.text(center_xy[0], center_xy[1], str(number), fontsize=LABEL_FONT_SIZE, ha="center", va="center", color=color,
            fontweight="bold", zorder=10)


def main() -> None:
    gap = GAP_SHOWN_MM

    # Axial coordinates, up the page: the front face of the front plate is 0, the robot is at the top.
    front_bottom, front_top = 0.0, FRONT_PLATE_THICKNESS_MM
    back_bottom = front_top + gap
    back_top = back_bottom + BACK_PLATE_THICKNESS_MM
    sflange_bottom, sflange_top = back_top, back_top + SPIGOT_FLANGE_THICKNESS_MM
    adapter_bottom = sflange_top                       # the spigot flange seats against the adapter's face
    adapter_top = adapter_bottom + ADAPTER_THICKNESS_MM
    robot_flange_top = adapter_top + FLANGE_THICKNESS_MM
    bore_top = adapter_bottom + BORE_DEPTH_MM
    barrel_top = sflange_top + SPIGOT_LENGTH_MM
    bore_r, hole_r = BORE_DIAMETER_MM / 2.0, CROSS_HOLE_DIAMETER_MM / 2.0
    adapter_r = ADAPTER_WIDTH_MM / 2.0
    pin_y = adapter_bottom + CROSS_HOLE_HEIGHT_MM
    dowel_y = sflange_top + DOWEL_HEIGHT_MM

    fig, ax = plt.subplots(figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI)

    # Target plates and standoffs.
    plate_style = dict(facecolor=SKY_BLUE, edgecolor=BLACK, lw=1.3)
    broken_slab(ax, front_bottom, front_top, PLATE_HALF_WIDTH_DRAWN_MM, **plate_style)
    broken_slab(ax, back_bottom, back_top, PLATE_HALF_WIDTH_DRAWN_MM, **plate_style)
    for sign in (-1.0, 1.0):
        ax.add_patch(Rectangle((sign * STANDOFF_RADIUS_MM - STANDOFF_DIAMETER_MM / 2.0, front_top),
                               STANDOFF_DIAMETER_MM, gap, facecolor=BLUISH_GREEN, edgecolor=BLACK, lw=1.2, zorder=3))

    # Spigot: flange against the back plate, barrel up into the adapter's bore.
    steel = dict(facecolor=LIGHT_GRAY, edgecolor=BLACK, lw=1.3)
    ax.add_patch(Rectangle((-SPIGOT_FLANGE_DIAMETER_MM / 2.0, sflange_bottom), SPIGOT_FLANGE_DIAMETER_MM,
                           SPIGOT_FLANGE_THICKNESS_MM, zorder=3, **steel))
    ax.add_patch(Rectangle((-bore_r, sflange_top), BORE_DIAMETER_MM, SPIGOT_LENGTH_MM, zorder=3, **steel))

    # Adapter with its bore, the keyway (left) and the cross hole, drawn as the adapter's section minus the voids.
    ax.add_patch(Rectangle((-adapter_r, adapter_bottom), ADAPTER_WIDTH_MM, ADAPTER_THICKNESS_MM,
                           facecolor=ORANGE, alpha=0.6, edgecolor=BLACK, lw=1.3, zorder=2))
    ax.add_patch(Rectangle((-adapter_r, adapter_bottom), ADAPTER_WIDTH_MM, ADAPTER_THICKNESS_MM, facecolor="none",
                           edgecolor=BLACK, lw=1.3, zorder=4))
    ax.add_patch(Rectangle((-bore_r, adapter_bottom), BORE_DIAMETER_MM, BORE_DEPTH_MM, facecolor="white",
                           edgecolor=BLACK, lw=1.0, zorder=2.5))
    ax.add_patch(Rectangle((-bore_r - KEYWAY_DEPTH_MM, adapter_bottom), KEYWAY_DEPTH_MM, BORE_DEPTH_MM,
                           facecolor="white", edgecolor=BLACK, lw=1.0, zorder=2.5))
    # The part of the spigot barrel and the dowel are redrawn above the voids (zorder 3 and 4 vs 2.5).
    ax.add_patch(Rectangle((-bore_r, sflange_top), BORE_DIAMETER_MM, SPIGOT_LENGTH_MM, zorder=3.2, **steel))
    ax.add_patch(Rectangle((-bore_r - KEYWAY_DEPTH_MM, dowel_y - DOWEL_DIAMETER_MM / 2.0), KEYWAY_DEPTH_MM,
                           DOWEL_DIAMETER_MM, facecolor=VERMILLION, edgecolor=BLACK, lw=1.0, zorder=3.4))

    # Robot flange.
    ax.add_patch(Rectangle((-FLANGE_DIAMETER_MM / 2.0, adapter_top), FLANGE_DIAMETER_MM, FLANGE_THICKNESS_MM,
                           facecolor=GRAY, edgecolor=BLACK, lw=1.3, zorder=2))

    # Ball-lock pin through the cross hole (horizontal), handle at the right.
    pin_left, pin_right = -adapter_r, adapter_r + PIN_END_BEYOND_ADAPTER_MM
    ax.add_patch(Rectangle((pin_left, pin_y - hole_r), pin_right - pin_left, CROSS_HOLE_DIAMETER_MM,
                           facecolor=REDDISH_PURPLE, edgecolor=BLACK, lw=1.0, zorder=5))
    handle_w, handle_l = PIN_HANDLE_SIZE_MM
    ax.add_patch(Rectangle((pin_right, pin_y - handle_l / 2.0), handle_w, handle_l, facecolor=REDDISH_PURPLE,
                           edgecolor=BLACK, lw=1.0, zorder=5))

    # Spigot axis (the datum axis), as a center line along the whole stack.
    ax.plot([0, 0], [front_bottom - 10.0, robot_flange_top + 10.0], color=GRAY, lw=0.9, linestyle=CENTERLINE_DASHES,
            zorder=6)

    # The gap G as a dimension, to the left of the plates.
    x_dim = -PLATE_HALF_WIDTH_DRAWN_MM - 12.0
    ax.annotate("", xy=(x_dim, front_top), xytext=(x_dim, back_bottom),
                arrowprops=dict(arrowstyle="<->", lw=1.2, color=VERMILLION, shrinkA=0, shrinkB=0))
    for y in (front_top, back_bottom):
        ax.plot([x_dim - 4.0, -PLATE_HALF_WIDTH_DRAWN_MM + 4.0], [y, y], color=VERMILLION, lw=0.7, linestyle=":")
    ax.text(x_dim - 6.0, (front_top + back_bottom) / 2.0,
            f"gap G\n{gap:g} mm shown\n({GAP_OTHER_MM:g} mm with the\nlong standoffs)", fontsize=LABEL_FONT_SIZE,
            color=VERMILLION, ha="right", va="center", fontweight="bold")

    # Part numbers on the drawing (the key at the right names them), the datum and the axis.
    # The numbers stand CALLOUT_SPACING_MM apart or more in the column; the leaders slant to reach their parts.
    callout(ax, 1, (CALLOUT_X_MM, adapter_top + FLANGE_THICKNESS_MM / 2.0 + CALLOUT_SPACING_MM / 3.0),
            (FLANGE_DIAMETER_MM / 2.0, adapter_top + FLANGE_THICKNESS_MM / 2.0))
    callout(ax, 2, (CALLOUT_X_MM, pin_y + 1.2 * CALLOUT_SPACING_MM), (adapter_r, adapter_top - 4.0))
    callout(ax, 3, (CALLOUT_X_MM, pin_y), (pin_right + handle_w, pin_y))
    callout(ax, 4, (CALLOUT_X_MM, sflange_top - 1.0), (bore_r, sflange_top + 4.0))
    callout(ax, 6, (CALLOUT_X_MM, (back_bottom + back_top) / 2.0 + 1.5), (PLATE_HALF_WIDTH_DRAWN_MM - 6.0,
                                                                          (back_bottom + back_top) / 2.0))
    callout(ax, 7, (CALLOUT_X_MM, front_top + gap / 2.0 - 1.5), (STANDOFF_RADIUS_MM + STANDOFF_DIAMETER_MM / 2.0,
                                                                 front_top + gap / 2.0))
    callout(ax, 8, (CALLOUT_X_MM, front_bottom + FRONT_PLATE_THICKNESS_MM / 2.0 - 1.5),
            (PLATE_HALF_WIDTH_DRAWN_MM - 6.0, front_bottom + FRONT_PLATE_THICKNESS_MM / 2.0))
    callout(ax, 5, (DOWEL_CALLOUT_X_MM, dowel_y), (-bore_r - KEYWAY_DEPTH_MM / 2.0, dowel_y), color=VERMILLION)
    ax.text(DOWEL_CALLOUT_X_MM - CALLOUT_RADIUS_MM - 3.0, dowel_y, "dowel datum:\nthe dowel and\nthe spigot axis",
            fontsize=LABEL_FONT_SIZE, color=VERMILLION, ha="right", va="center", fontweight="bold")
    ax.text(3.0, robot_flange_top + 9.0, "spigot axis", fontsize=SMALL_FONT_SIZE, color=GRAY, ha="left", va="center")

    # The key: number, name and drawing number, size.
    key = [
        (1, f"Robot flange, ISO 9409-1-50-4-M6, {FLANGE_DIAMETER_MM:g} mm diameter."),
        (2, f"Target adapter PT-01, aluminum, {ADAPTER_WIDTH_MM:g} mm wide x {ADAPTER_THICKNESS_MM:g} mm thick, with a "
            f"{BORE_DIAMETER_MM:g} mm bore {BORE_DEPTH_MM:g} mm deep, a {KEYWAY_WIDTH_MM:g} mm keyway and an "
            f"{CROSS_HOLE_DIAMETER_MM:g} mm cross hole through the bore. It stays on the flange."),
        (3, f"Ball-lock pin, {CROSS_HOLE_DIAMETER_MM:g} mm, through the adapter's cross hole and the spigot's."),
        (4, f"Spigot PT-02, steel, {SPIGOT_DIAMETER_MM:g} mm diameter x {SPIGOT_LENGTH_MM:g} mm long, with a "
            f"{SPIGOT_FLANGE_DIAMETER_MM:g} mm diameter x {SPIGOT_FLANGE_THICKNESS_MM:g} mm flange, bolted to the back "
            f"plate (4 x M5). One per feature target."),
        (5, f"Dowel, {DOWEL_DIAMETER_MM:g} mm, radial on the spigot, in the keyway: it sets the target's orientation."),
        (6, f"Back plate, {BACK_PLATE_THICKNESS_MM:g} mm."),
        (7, f"Standoffs PT-03, {STANDOFF_DIAMETER_MM:g} mm diameter, {gap:g} mm long as drawn ({GAP_OTHER_MM:g} mm for "
            "the large gap); one at each corner of the plate, two shown."),
        (8, f"Front plate, {FRONT_PLATE_THICKNESS_MM:g} mm: the face the sensor sees."),
    ]
    y = robot_flange_top + 28.0
    for number, text in key:
        lines = textwrap.wrap(text, KEY_WRAP_CHARACTERS)
        callout_xy = (KEY_X_MM, y - KEY_LINE_HEIGHT_MM / 2.0)
        ax.add_patch(Circle(callout_xy, CALLOUT_RADIUS_MM * 0.8, facecolor="white", edgecolor=BLACK, lw=1.0))
        ax.text(callout_xy[0], callout_xy[1], str(number), fontsize=SMALL_FONT_SIZE + 0.5, ha="center", va="center",
                fontweight="bold")
        ax.text(KEY_X_MM + CALLOUT_RADIUS_MM + 2.5, y, "\n".join(lines), fontsize=LABEL_FONT_SIZE, ha="left",
                va="top", linespacing=1.25)
        y -= len(lines) * KEY_LINE_HEIGHT_MM + KEY_ITEM_GAP_MM

    # Direction cues.
    ax.text(0.0, front_bottom - 12.0, "toward the sensor", fontsize=LABEL_FONT_SIZE, color=BLACK, ha="center",
            va="top", style="italic")
    ax.text(0.0, robot_flange_top + 20.0, "toward the robot", fontsize=LABEL_FONT_SIZE, color=BLACK, ha="center",
            va="bottom", style="italic")

    ax.set_xlim(X_LIMITS_MM[0], X_LIMITS_MM[1])
    ax.set_ylim(front_bottom - 24.0, robot_flange_top + 38.0)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.subplots_adjust(left=0.005, right=0.995, top=0.995, bottom=0.005)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, flange_diameter_mm=FLANGE_DIAMETER_MM, adapter_width_mm=ADAPTER_WIDTH_MM,
                  adapter_thickness_mm=ADAPTER_THICKNESS_MM, bore_diameter_mm=BORE_DIAMETER_MM,
                  bore_depth_mm=BORE_DEPTH_MM, keyway_width_mm=KEYWAY_WIDTH_MM,
                  cross_hole_diameter_mm=CROSS_HOLE_DIAMETER_MM, spigot_diameter_mm=SPIGOT_DIAMETER_MM,
                  spigot_length_mm=SPIGOT_LENGTH_MM, spigot_flange_diameter_mm=SPIGOT_FLANGE_DIAMETER_MM,
                  spigot_flange_thickness_mm=SPIGOT_FLANGE_THICKNESS_MM, dowel_diameter_mm=DOWEL_DIAMETER_MM,
                  back_plate_thickness_mm=BACK_PLATE_THICKNESS_MM, front_plate_thickness_mm=FRONT_PLATE_THICKNESS_MM,
                  standoff_diameter_mm=STANDOFF_DIAMETER_MM, gap_shown_mm=gap)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
