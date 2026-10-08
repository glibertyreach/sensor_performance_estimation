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
more across and are drawn cut off with break lines.

    python3 docs/procedures/figures/make_fig_mounting.py      (from the repository root)
"""
from __future__ import annotations

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
FIGURE_SIZE_IN = (13.0, 7.0)
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
BREAK_TOOTH_MM = 4.0                      # size of the zigzag break line
PIN_END_BEYOND_ADAPTER_MM = 26.0          # how far the pin's handle end sticks out of the adapter
PIN_HANDLE_SIZE_MM = (8.0, 22.0)          # axial width, radial length of the T-handle
LABEL_COLUMN_MM = 108.0                   # label text starts this far from the axis
LABEL_FONT_SIZE = 8.5
SMALL_FONT_SIZE = 7.5
LEADER_LW = 0.8
CENTERLINE_DASHES = (0, (8, 3, 1.5, 3))


def broken_slab(ax, bottom: float, top: float, half_width: float, **style) -> None:
    """A plate seen edge-on, cut off at both ends with a zigzag break line."""
    teeth = int((top - bottom) / BREAK_TOOTH_MM)
    ys = [bottom + (top - bottom) * i / max(teeth, 1) for i in range(teeth + 1)]
    right = [(half_width + (BREAK_TOOTH_MM * 0.5 if i % 2 else -BREAK_TOOTH_MM * 0.5), y) for i, y in enumerate(ys)]
    left = [(-half_width + (BREAK_TOOTH_MM * 0.5 if i % 2 else -BREAK_TOOTH_MM * 0.5), y) for i, y in enumerate(ys)]
    ax.add_patch(Polygon(right + left[::-1], closed=True, **style))


def leader(ax, text: str, label_xy, target_xy, side: str, color: str = BLACK, size: float = LABEL_FONT_SIZE) -> None:
    """Text at label_xy with a thin leader line to target_xy; side is 'left' or 'right' of the drawing."""
    ax.annotate(text, xy=target_xy, xytext=label_xy, fontsize=size, color=color,
                ha="left" if side == "right" else "right", va="center",
                arrowprops=dict(arrowstyle="-", color=color, lw=LEADER_LW, shrinkA=2, shrinkB=0,
                                connectionstyle="arc,angleA=180,angleB=0,armA=6,armB=0,rad=0"
                                if side == "right" else
                                "arc,angleA=0,angleB=180,armA=6,armB=0,rad=0"))


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

    # Labels on the right: spigot, adapter, pin, robot flange. Left: plates, standoff, dowel.
    xr, xl = LABEL_COLUMN_MM, -LABEL_COLUMN_MM
    leader(ax, f"robot flange, ISO 9409-1-50-4-M6\n({FLANGE_DIAMETER_MM:g} mm diameter)",
           (xr, adapter_top + FLANGE_THICKNESS_MM / 2.0 + 6.0), (FLANGE_DIAMETER_MM / 2.0, adapter_top + FLANGE_THICKNESS_MM / 2.0),
           "right")
    leader(ax, f"target adapter PT-01, aluminum\n{ADAPTER_WIDTH_MM:g} wide x {ADAPTER_THICKNESS_MM:g} thick, stays on the flange",
           (xr, adapter_top - 5.0), (adapter_r, adapter_top - 6.0), "right", color=BLACK)
    leader(ax, f"ball-lock pin, {CROSS_HOLE_DIAMETER_MM:g} mm, through the adapter's\ncross hole and the spigot's cross hole",
           (xr, pin_y + 2.0), (pin_right + handle_w, pin_y), "right", color=BLACK)
    leader(ax, f"spigot PT-02, steel: {SPIGOT_DIAMETER_MM:g} mm diameter x {SPIGOT_LENGTH_MM:g} mm long",
           (xr, sflange_top + SPIGOT_LENGTH_MM * 0.85), (bore_r, sflange_top + SPIGOT_LENGTH_MM * 0.85), "right")
    leader(ax, f"spigot flange, {SPIGOT_FLANGE_DIAMETER_MM:g} mm diameter x {SPIGOT_FLANGE_THICKNESS_MM:g} mm,\n"
               "bolted to the back plate (4 x M5)",
           (xr, sflange_bottom + SPIGOT_FLANGE_THICKNESS_MM / 2.0 - 4.0),
           (SPIGOT_FLANGE_DIAMETER_MM / 2.0, sflange_bottom + SPIGOT_FLANGE_THICKNESS_MM / 2.0), "right")
    leader(ax, f"back plate, {BACK_PLATE_THICKNESS_MM:g} mm (aluminum)", (xr, (back_bottom + back_top) / 2.0 - 3.0),
           (PLATE_HALF_WIDTH_DRAWN_MM - 5.0, (back_bottom + back_top) / 2.0), "right")
    leader(ax, f"standoff PT-03, {STANDOFF_DIAMETER_MM:g} mm diameter, one at each corner (two shown)",
           (xr, (front_top + back_bottom) / 2.0 - 2.0), (STANDOFF_RADIUS_MM + STANDOFF_DIAMETER_MM / 2.0,
                                                        (front_top + back_bottom) / 2.0), "right")
    leader(ax, f"front plate, {FRONT_PLATE_THICKNESS_MM:g} mm (aluminum), the face the sensor sees",
           (xr, front_bottom + FRONT_PLATE_THICKNESS_MM / 2.0 - 3.0),
           (PLATE_HALF_WIDTH_DRAWN_MM - 5.0, front_bottom + FRONT_PLATE_THICKNESS_MM / 2.0), "right")
    leader(ax, f"bore {BORE_DIAMETER_MM:g} mm x {BORE_DEPTH_MM:g} mm deep\nwith a {KEYWAY_WIDTH_MM:g} mm keyway",
           (xl, bore_top + 12.0), (-bore_r - KEYWAY_DEPTH_MM, bore_top - 4.0), "left")
    leader(ax, f"dowel, {DOWEL_DIAMETER_MM:g} mm, radial, in the keyway\nDOWEL DATUM: the dowel with the spigot axis,\n"
               "the same for every target (section 2)",
           (xl, dowel_y + 3.0), (-bore_r - KEYWAY_DEPTH_MM, dowel_y), "left", color=VERMILLION)
    ax.text(3.0, robot_flange_top + 11.0, "spigot axis", fontsize=SMALL_FONT_SIZE, color=GRAY, ha="left", va="center")

    # Direction cues.
    ax.text(0.0, front_bottom - 16.0, "toward the sensor", fontsize=LABEL_FONT_SIZE, color=BLACK, ha="center", va="top",
            style="italic")
    ax.text(0.0, robot_flange_top + 20.0, "toward the robot", fontsize=LABEL_FONT_SIZE, color=BLACK, ha="center",
            va="bottom", style="italic")

    extent_x = LABEL_COLUMN_MM + 125.0
    ax.set_xlim(-extent_x, extent_x)
    ax.set_ylim(front_bottom - 30.0, robot_flange_top + 34.0)
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
