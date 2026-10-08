"""Shared helpers for the target shop drawings PT-04 to PT-07.

This module holds what the four target drawings have in common:

* the rules of the characterization specification (Sections 3.2 and 3.3) as
  named constants: 45 degree back bevel, 0.1 mm land, plate thicknesses, finish;
* loading of the target definitions from the code (feature positions and sizes
  are never typed in a drawing script);
* the rounding rule for plate outlines;
* drawing helpers used by more than one sheet: the spigot hole pattern in a plan
  view, the 4:1 knife-edge detail, the standoff joint in section, and section
  regions that end in a break line.

Section views in the PT-04 to PT-07 sheets use u (horizontal) and z (vertical,
toward the sensor, so the front face is at the top).
"""

from __future__ import annotations

import math
import os
import sys

# The scripts must run from any working directory: make the sibling modules and
# the repository root (for ``sensorperf``) importable.
HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
for _p in (HERE, REPO_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import drafting as d  # noqa: E402
import spigot_pattern as sp  # noqa: E402
from drafting import Sheet, View, polar, thread_minor_diameter  # noqa: E402
from sensorperf.geometry.targets import make_standard_target_set  # noqa: E402
from sensorperf.parameters import CharacterizationParameters, SensorGeometry  # noqa: E402

# ---------------------------------------------------------------------------
# Rules of the characterization specification, Sections 3.2 and 3.3
# ---------------------------------------------------------------------------
BEVEL_DEG = 45.0  # bevel angle measured from the plate normal (90 degrees included for a countersink)
EDGE_LAND_MM = 0.1  # nominal flat land left at the front face of every knife edge
EDGE_LAND_TOL_MM = 0.1  # land tolerance, +0.1 / -0.1
EDGE_LAND_MAX_MM = 0.2  # the specification's upper limit of the land
FRONT_PLATE_THICKNESS_MM = 6.0  # front plates, the raised square
BACK_PLATE_THICKNESS_MM = 8.0  # every back plate (carries the spigot, or is plain)
FLATNESS_MM = 0.05  # flatness over each front face
PLATE_ROUND_STEP_MM = 1.0  # rounding rule: plate outlines are rounded UP to whole millimeters
HIDE_RAY_ANGLE_DEG = 30.0  # steepest viewing ray used for the hidden-standoff check
HIDE_MARGIN_MM = 5.0  # extra margin of the hidden-standoff check
POSITION_TOL_MM = 0.1  # position tolerance of the sites of a plate
SITE_DIAMETER_TOL_MM = 0.02  # tolerance of the front-face diameter of a disk or cutout

assert EDGE_LAND_MM + EDGE_LAND_TOL_MM <= EDGE_LAND_MAX_MM + 1e-9, "land tolerance exceeds the 0.2 mm limit"

# Tapped M5 holes for standoff studs and bracket screws (blind depths: spigot_pattern.py)
M5_MAJOR_MM = sp.SPIGOT_SCREW_DIAMETER_MM  # M5 major diameter
M5_MINOR_MM = thread_minor_diameter(M5_MAJOR_MM, sp.M5_PITCH_MM)  # M5 minor diameter (ISO 262)
M5_TAP_DRILL_MM = 4.2  # tap drill for M5 x 0.8
FRONT_STUD_RECESS_MM = FRONT_PLATE_THICKNESS_MM - sp.STANDOFF_FRONT_STUD_LENGTH_MM  # stud end below the front face (0.5)
assert abs(FRONT_STUD_RECESS_MM - 0.5) < 1e-9, "front stud must end 0.5 mm below the front face of a 6 mm plate"

EDGE_DETAIL_EXTRA_MM = 4.0  # material drawn beyond the end of the bevel in the 4:1 detail

# Text blocks shared by the sheets
MATERIAL_TEXT = "Aluminum tooling plate (MIC-6 or 6061-T6), finish-ground"
FINISH_TEXT = "Fine glass-bead blast, uniform, every face the sensor sees"
LAND_TEXT = f"{EDGE_LAND_MM:g} +{EDGE_LAND_TOL_MM:g}/−{EDGE_LAND_TOL_MM:g}"
NOTE_SIZE = 9.0  # smallest permitted note size

# Standard notes, as full sentences (each sheet picks the ones that apply)
NOTE_KNIFE_EDGE = (
    f"Every knife edge is beveled or countersunk FROM THE BACK at {BEVEL_DEG:g}° "
    f"({2 * BEVEL_DEG:g}° included for countersinks), leaving a flat land at the front face of "
    f"{LAND_TEXT} mm (never more than {EDGE_LAND_MAX_MM:g} mm). Measure the land of every edge and "
    f"record it in the as-built record."
)
NOTE_FINISH = (
    "Fine glass-bead blast, uniform, on every face the sensor can see. All plates and disks "
    "blasted in one batch, same medium and pressure."
)
NOTE_FLATNESS = f"Flatness {FLATNESS_MM:g} over each front face."
NOTE_SPIGOT = (
    f"Spigot PT-02: 4 \u00d7 {sp.SPIGOT_SCREW} tapped through holes in the back face on a \u00d8{sp.SPIGOT_PCD_MM:g} pitch circle at "
    f"{', '.join(f'{a:g}' for a in sp.SPIGOT_HOLE_ANGLES_DEG[:-1])} and {sp.SPIGOT_HOLE_ANGLES_DEG[-1]:g}\u00b0 "
    f"from +x, centered on the spigot axis; engrave an arrow {sp.ORIENTATION_MARK_WIDTH_MM:g} wide and "
    f"{sp.ORIENTATION_MARK_DEPTH_MM:g} deep pointing {sp.SPIGOT_DOWEL_DIRECTION} (the spigot's dowel direction)."
)
NOTE_STANDOFF_FRONT_THROUGH = (
    f"Standoffs PT-03, 15 mm set or 60 mm set; both sets delivered. Front plate: {sp.STANDOFF_STUD} through-tapped holes; "
    f"the {sp.STANDOFF_FRONT_STUD_LENGTH_MM:g} mm front stud ends 0.5 below the front face. Nothing may protrude the front "
    f"face; the plate is bead-blasted before assembly. Back plate: {sp.STANDOFF_STUD} through-tapped for the "
    f"{sp.STANDOFF_STUD_LENGTH_MM:g} mm back studs, which must not stand proud of its back face."
)


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def fmt(x: float) -> str:
    """Format a dimension without trailing zeros (16.0 -> '16', 5.9 -> '5.9')."""
    return f"{x:g}"


def fmt2(x: float) -> str:
    """Format a dimension with two decimals (6.975 -> '6.98')."""
    return f"{x:.2f}"


def round_up_mm(x: float, step: float = PLATE_ROUND_STEP_MM) -> float:
    """Rounding rule for plate outlines: round UP to the next whole step (167.97 x 2 -> 336)."""
    return math.ceil(x / step - 1e-9) * step


def rotate(u: float, v: float, deg: float) -> tuple[float, float]:
    """Rotate the point (u, v) counterclockwise by ``deg`` about the origin."""
    a = math.radians(deg)
    return (u * math.cos(a) - v * math.sin(a), u * math.sin(a) + v * math.cos(a))


def bevel_run(thickness: float, land: float = EDGE_LAND_MM, bevel_deg: float = BEVEL_DEG) -> float:
    """Horizontal run of the back bevel of a part of the given thickness (45 degrees: thickness - land)."""
    return (thickness - land) * math.tan(math.radians(bevel_deg))


def hidden_threshold_mm(gap_mm: float) -> float:
    """A standoff axis must be farther than this from the nearest opening edge (gap x tan 30 + 5 mm)."""
    return gap_mm * math.tan(math.radians(HIDE_RAY_ANGLE_DEG)) + HIDE_MARGIN_MM


def hidden_threshold_strict_mm(gap_mm: float, front_thickness_mm: float) -> float:
    """Stricter form of the check: the ray that grazes the front-face edge of the opening drops the front part's
    thickness as well, so it is ((gap + thickness) x tan 30 + 5 mm) from the opening edge at the back plate."""
    return (gap_mm + front_thickness_mm) * math.tan(math.radians(HIDE_RAY_ANGLE_DEG)) + HIDE_MARGIN_MM


def distance_to_square(point: tuple[float, float], side: float, rot_deg: float,
                       center: tuple[float, float] = (0.0, 0.0)) -> float:
    """Distance from a point to the region of a square of the given side rotated counterclockwise by rot_deg
    about ``center`` (0 inside the square)."""
    u, v = rotate(point[0] - center[0], point[1] - center[1], -rot_deg)
    qu, qv = max(abs(u) - side / 2, 0.0), max(abs(v) - side / 2, 0.0)
    return math.hypot(qu, qv)


def load_targets():
    """Return (params, target set) built exactly as the planner builds them."""
    params = CharacterizationParameters()
    return params, make_standard_target_set(params, SensorGeometry.indicative())


def png_size(path: str) -> tuple[int, int]:
    """Return (width, height) in pixels of a PNG file."""
    import matplotlib.image as mpimg

    h, w = mpimg.imread(path).shape[:2]
    return w, h


def report_sheet(number: str, path: str, issues: list[str]) -> None:
    """Print the outcome of one sheet: size and the layout problems found."""
    w, h = png_size(path)
    print(f"{number}: {os.path.basename(path)} {w} x {h} px, {len(issues)} layout problem(s)")
    for i in issues:
        print("    ", i)


# ---------------------------------------------------------------------------
# Plan-view helper: the spigot pattern on the back face (hidden in the front view)
# ---------------------------------------------------------------------------
def orientation_arrow(center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    """Outline of the engraved orientation arrow (target frame), pointing +y, just outside the flange."""
    cx, cy = center
    y0 = cy + sp.SPIGOT_FLANGE_DIAMETER_MM / 2 + sp.ORIENTATION_MARK_CLEARANCE_MM  # tail
    y1 = y0 + sp.ORIENTATION_MARK_LENGTH_MM  # tip
    head = sp.ORIENTATION_MARK_WIDTH_MM  # head length (equal to its width)
    hw, sw = sp.ORIENTATION_MARK_WIDTH_MM / 2, sp.ORIENTATION_MARK_WIDTH_MM / 4  # head / shaft half widths
    return [(cx - sw, y0), (cx + sw, y0), (cx + sw, y1 - head), (cx + hw, y1 - head), (cx, y1),
            (cx - hw, y1 - head), (cx - sw, y1 - head)]


def spigot_hole_points(center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    """Centers of the four spigot screw holes."""
    return [polar(center, sp.SPIGOT_PCD_MM / 2, a) for a in sp.SPIGOT_HOLE_ANGLES_DEG]


def plan_spigot_pattern(view: View, center: tuple[float, float] = (0.0, 0.0), style: str = "hidden") -> None:
    """Draw the spigot pattern of the back face in a front view: flange (phantom), pitch circle, four M5
    holes and the orientation arrow, all as hidden detail."""
    view.circle(center, sp.SPIGOT_FLANGE_DIAMETER_MM / 2, "phantom")
    view.circle(center, sp.SPIGOT_PCD_MM / 2, "center")
    for p in spigot_hole_points(center):
        view.circle(p, M5_MAJOR_MM / 2, style)
        view.center_cross(p, M5_MAJOR_MM / 2)
    view.polyline(orientation_arrow(center), style, closed=True)
    view.center_cross(center, 3.0)


# ---------------------------------------------------------------------------
# Section helpers
# ---------------------------------------------------------------------------
def region(view: View, pts: list[tuple[float, float]], other: bool = False,
           crop_u: tuple[float, ...] = ()) -> None:
    """Hatch a closed section polygon and outline it, leaving out vertical edges that lie on a crop line
    (a break line is drawn there instead)."""
    view.hatch(pts, other)
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        if abs(a[0] - b[0]) < 1e-9 and any(abs(a[0] - c) < 1e-9 for c in crop_u):
            continue
        view.line(a, b, "outline")


def break_line(view: View, u: float, z0: float, z1: float) -> None:
    """Freehand break line at model u between heights z0 and z1."""
    view.sheet.wavy(view.P(u, z0), view.P(u, z1))


def notched_edge(u_from: float, u_to: float, z: float, hole_us: list[float], hole_w: float, depth: float,
                 up: bool) -> list[tuple[float, float]]:
    """Points along a horizontal edge from u_from to u_to at height z, with a rectangular notch of width
    ``hole_w`` and ``depth`` (toward +z when ``up``) at each hole center.  Includes both ends."""
    sgn = 1.0 if up else -1.0
    lo, hi = min(u_from, u_to), max(u_from, u_to)
    pts: list[tuple[float, float]] = [(lo, z)]
    for u0 in sorted(h for h in hole_us if lo < h < hi):
        pts += [(u0 - hole_w / 2, z), (u0 - hole_w / 2, z + sgn * depth),
                (u0 + hole_w / 2, z + sgn * depth), (u0 + hole_w / 2, z)]
    pts.append((hi, z))
    return pts if u_from <= u_to else pts[::-1]


def back_plate_piece(u_left: float, u_right: float, hole_us: list[float], thickness: float = BACK_PLATE_THICKNESS_MM
                     ) -> list[tuple[float, float]]:
    """Section of a back plate (front face at z = 0, back face at z = -thickness) with M5 through holes."""
    top = notched_edge(u_left, u_right, 0.0, hole_us, M5_MAJOR_MM, thickness, up=False)
    return top + [(u_right, -thickness), (u_left, -thickness)]


def standoff_piece(u0: float, gap: float) -> list[tuple[float, float]]:
    """Section of one standoff on axis u0: body of the gap length, 8 mm stud into the back plate (through
    hole, flush with its back face) and the 5.5 mm front stud into the front part."""
    r_body, r_stud = sp.STANDOFF_BODY_DIAMETER_MM / 2, M5_MAJOR_MM / 2
    low = sp.STANDOFF_STUD_LENGTH_MM  # stud in the back plate
    up = sp.STANDOFF_FRONT_STUD_LENGTH_MM  # stud in the front part
    return [(u0 - r_stud, -low), (u0 + r_stud, -low), (u0 + r_stud, 0.0), (u0 + r_body, 0.0),
            (u0 + r_body, gap), (u0 + r_stud, gap), (u0 + r_stud, gap + up), (u0 - r_stud, gap + up),
            (u0 - r_stud, gap), (u0 - r_body, gap), (u0 - r_body, 0.0), (u0 - r_stud, 0.0)]


# ---------------------------------------------------------------------------
# The 4:1 knife-edge detail (used by PT-04, PT-05, PT-06 and PT-07)
# ---------------------------------------------------------------------------
def draw_edge_detail(sh: Sheet, origin: tuple[float, float], scale: float, thickness: float, material_side: int,
                     title: str, subtitle: str = "") -> dict[str, float]:
    """Draw a section through one knife edge, perpendicular to the edge.

    ``origin`` is the paper position of the front-face edge point (u = 0, z = 0).  ``material_side`` is +1
    when the material lies at larger u (a countersunk hole or window: the opening is at smaller u) and -1
    when it lies at smaller u (a disk or the raised square: the opening is at larger u).  The bevel always
    runs from the end of the land into the material as it goes back, so the part narrows away from the
    sensor for a frustum and the opening widens away from the sensor for a countersink.

    Returns the derived values (bevel run, back face offset).
    """
    m = material_side
    run = bevel_run(thickness)
    span = run + EDGE_DETAIL_EXTRA_MM  # material shown beyond the end of the bevel
    v = View(sh, origin, scale)
    poly = [(0.0, 0.0), (m * span, 0.0), (m * span, -thickness), (m * run, -thickness), (0.0, -EDGE_LAND_MM)]
    region(v, poly, crop_u=(m * span,))
    break_line(v, m * span, 0.0, -thickness)
    open_sgn = -m  # the side of the opening
    # front and back face labels
    sh.text(origin[0] - open_sgn * 2.0, origin[1] + 2.0, "FRONT FACE (SENSOR SIDE)", size=d.FONT_NOTE,
            ha="left" if open_sgn < 0 else "right")
    # thickness: vertical dimension on the opening side
    off = 20.0 * open_sgn
    v.dim_v(0.0, 0.0, 0.0, -thickness, off, fmt(thickness), base_u=0.0)
    # horizontal run of the bevel, below the back face
    v.dim_h(0.0, m * run, -thickness, -thickness, -9.0, fmt(round(run, 2)), ext0=True, ext1=True)
    # the land: leader to the land with its tolerance
    v.leader((0.0, -EDGE_LAND_MM / 2), f"LAND {LAND_TEXT}", open_sgn * 16.0, 8.0, terminator="dot")
    # bevel angle from the plate normal
    ang0 = -90.0
    ang1 = math.degrees(math.atan2(-(thickness - EDGE_LAND_MM), m * run))
    lo, hi = sorted((ang0, ang1))
    c0 = (0.0, -EDGE_LAND_MM)
    v.line(c0, (0.0, -EDGE_LAND_MM - (thickness - EDGE_LAND_MM) * 0.75), "thin")  # plate normal
    r_paper = scale * (thickness - EDGE_LAND_MM) * 0.55
    mid_x = r_paper * math.cos(math.radians((lo + hi) / 2))  # paper x of the arc midpoint relative to the arc center
    v.dim_angle(c0, lo, hi, r_paper, f"{BEVEL_DEG:g}\u00b0", text_dx=open_sgn * 7.0 - mid_x, text_dy=-3.0)
    sh.text(origin[0], origin[1] - scale * thickness - 17.0, title, size=d.FONT_LABEL, ha="center", weight="bold")
    if subtitle:
        sh.text(origin[0], origin[1] - scale * thickness - 22.0, subtitle, size=d.FONT_NOTE, ha="center")
    return {"bevel_run_mm": run}
