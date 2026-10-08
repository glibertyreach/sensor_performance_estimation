"""Shop drawing PT-05: T3b square window.

Parts on this sheet: the front plate (300 x 300 x 6, 160 mm square window
rotated 5 degrees and countersunk from the back) and the plain back plate
(300 x 300 x 8, spigot pattern in its back face).  Four PT-03 standoffs at the
corners join them.

Coordinates
-----------
Plan (front view): target frame, origin at the plate center, x right, y up, seen
from the sensor.  Section A-A is an offset section: the cutting plane runs along
y = +STANDOFF_AXIS through the left and right standoffs and along y = 0 through
the window, and is viewed toward +y so that u = x runs to the right.  Only the
left half is drawn (the right half is the same, apart from the slant); z is the
height above the front face of the back plate.  The plane crosses the window
edges at the slant angle, so the edge positions in section are the true ones
divided by cos(slant); the true profile is Detail B.
"""

from __future__ import annotations

import math
import os

import drafting as d
import spigot_pattern as sp
import target_drawing_common as c
from drafting import Sheet, TitleInfo, View

# ---------------------------------------------------------------------------
# Part data from the code and the specification
# ---------------------------------------------------------------------------
PARAMS, TARGET_SET = c.load_targets()
T3B = TARGET_SET.targets["T3b"]
WINDOW = T3B.features[0]  # the square window feature
GAP_SMALL_MM = PARAMS.gap_small_mm
GAP_LARGE_MM = PARAMS.gap_large_mm
PLATE_W, PLATE_H = (c.round_up_mm(v) for v in TARGET_SET.derived["edge_plate_size_mm"])  # both plates, whole mm
WINDOW_SIDE = WINDOW.diameter_mm  # side of the square window
WINDOW_ROT = WINDOW.rotation_deg  # slant, counterclockwise seen from the sensor
FRONT_T = c.FRONT_PLATE_THICKNESS_MM  # front plate thickness (6)
BACK_T = c.BACK_PLATE_THICKNESS_MM  # back plate thickness (8)
CORNER_STANDOFF_INSET_MM = 15.0  # standoff axis distance from each plate edge
BODY_D = sp.STANDOFF_BODY_DIAMETER_MM  # standoff body diameter
SHOWN_GAP_MM = GAP_SMALL_MM  # the section shows the 15 mm set

# Derived dimensions (computed, never typed)
RUN = c.bevel_run(FRONT_T)  # horizontal run of the back countersink (5.9)
BACK_OPENING = WINDOW_SIDE + 2 * RUN  # side of the window opening in the back face of the front plate
AXIS = PLATE_W / 2 - CORNER_STANDOFF_INSET_MM  # standoff axes at (+/-AXIS, +/-AXIS)
assert PLATE_W == PLATE_H, "T3b plates are square"
STANDOFF_AXES = [(sx * AXIS, sy * AXIS) for sx in (-1, 1) for sy in (-1, 1)]
SECTION_COS = math.cos(math.radians(WINDOW_ROT))  # the y = 0 plane crosses the window edges at the slant


NEAREST_STANDOFF_MM = min(c.distance_to_square(p, WINDOW_SIDE, WINDOW_ROT) for p in STANDOFF_AXES)  # axis to window


def window_hidden_margins() -> dict[float, float]:
    """Margin (mm) of the nearest standoff axis beyond gap x tan 30 + 5 mm, for each gap."""
    return {g: NEAREST_STANDOFF_MM - c.hidden_threshold_mm(g) for g in (GAP_SMALL_MM, GAP_LARGE_MM)}


assert NEAREST_STANDOFF_MM > c.hidden_threshold_mm(GAP_LARGE_MM), "standoffs would be visible through the window"

# ---------------------------------------------------------------------------
# Sheet layout (paper mm)
# ---------------------------------------------------------------------------
PLAN_SCALE = 0.5  # plan at 1:2
PLAN_CENTER = (98.0, 166.0)  # paper position of the plate center
SEC_SCALE = 1.0  # section at 1:1
SEC_U_LEFT = -PLATE_W / 2  # left plate edge in the section
SEC_U_RIGHT = -40.0  # the half section is cropped here (inside the window)
SEC_ORIGIN = (372.0, 212.0)  # paper position of (u = 0, z = 0): front face of the back plate
DET_SCALE = 4.0  # knife-edge detail at 4:1
DET_ORIGIN = (346.0, 150.0)  # paper position of the front-face edge point of the detail
NOTES_X = 12.0
NOTES_TOP = 78.0
NOTES_W = 224.0
CALL_X = 190.0  # left end of the plan callout texts
CUT_END_U = PLATE_W / 2 + 8.0  # the cutting-plane line starts this far out (target frame)
CUT_JOG_X = 110.0  # |x| where the cutting plane steps from y = AXIS to y = 0
DIM_TOP = 7.0
DIM_LEFT = -8.0
LABEL_DY = -9.0


def build(out_dir: str) -> list[str]:
    sh = Sheet("PT-05")
    sh.border()
    plan = View(sh, PLAN_CENTER, PLAN_SCALE)
    hw, hh = PLATE_W / 2, PLATE_H / 2
    rot = WINDOW_ROT

    def wn(u, v):
        """Window-frame point -> target frame."""
        return c.rotate(u, v, rot)

    # ---------------- plan view ----------------
    plan.polyline([(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)], "outline", closed=True)
    h = WINDOW_SIDE / 2
    plan.polyline([wn(-h, -h), wn(h, -h), wn(h, h), wn(-h, h)], "outline", closed=True)
    hb = BACK_OPENING / 2
    plan.polyline([wn(-hb, -hb), wn(hb, -hb), wn(hb, hb), wn(-hb, hb)], "hidden", closed=True)
    for p in STANDOFF_AXES:
        plan.circle(p, BODY_D / 2, "hidden")
        plan.center_cross(p, BODY_D / 2)
    c.plan_spigot_pattern(plan)
    plan.center_h(-hw - 6, hw + 6, 0.0)
    plan.center_v(-hh - 6, hh + 6, 0.0)
    # offset cutting plane A-A: along y = AXIS through the standoffs, along y = 0 through the window
    plan.cutting_plane([(-CUT_END_U, AXIS), (-CUT_JOG_X, AXIS), (-CUT_JOG_X, 0.0), (CUT_JOG_X, 0.0),
                        (CUT_JOG_X, AXIS), (CUT_END_U, AXIS)], sight=(0.0, 1.0), label="A")
    plan.dim_h(-hw, hw, hh, hh, DIM_TOP, c.fmt(PLATE_W))
    plan.dim_v(-hw, -hw, -hh, hh, DIM_LEFT, c.fmt(PLATE_H))

    def callout(target, text, slot_y):
        t = plan.P(*target)
        return plan.leader(target, text, CALL_X - t[0] - d.LEADER_SHOULDER - d.LEADER_TEXT_GAP, slot_y - t[1])

    arrow = c.orientation_arrow()
    callout(arrow[3], f"ORIENTATION ARROW, BACK FACE OF THE BACK PLATE, POINTS {sp.SPIGOT_DOWEL_DIRECTION}:\n"
                      f"{c.fmt(sp.ORIENTATION_MARK_WIDTH_MM)} WIDE × {c.fmt(sp.ORIENTATION_MARK_DEPTH_MM)} DEEP", 174.0)
    callout(wn(h, 0.0), f"{c.fmt(WINDOW_SIDE)} × {c.fmt(WINDOW_SIDE)} SQUARE WINDOW, THROUGH,\n"
                        f"SLANT {c.fmt(rot)}° COUNTERCLOCKWISE,\nCOUNTERSUNK {c.fmt(2 * c.BEVEL_DEG)}° FROM THE BACK", 152.0)
    sp_pts = c.spigot_hole_points()
    callout((sp_pts[3][0] + c.M5_MAJOR_MM / 2, sp_pts[3][1]),
            f"4 × {sp.SPIGOT_SCREW} THRU IN THE BACK PLATE (SPIGOT PT-02),\n"
            f"Ø{c.fmt(sp.SPIGOT_PCD_MM)} PCD AT {', '.join(c.fmt(a) for a in sp.SPIGOT_HOLE_ANGLES_DEG)}° FROM +x,\n"
            f"CENTERED ON THE PLATE; Ø{c.fmt(sp.SPIGOT_FLANGE_DIAMETER_MM)} FLANGE (PHANTOM)", 130.0)
    callout(wn(hb, -hb * 0.55), f"BACK FACE OPENING {c.fmt(round(BACK_OPENING, 2))} × {c.fmt(round(BACK_OPENING, 2))} (HIDDEN),\n"
                                f"LAND {c.LAND_TEXT} AT THE FRONT FACE", 110.0)
    callout((STANDOFF_AXES[3][0] + BODY_D / 2, STANDOFF_AXES[3][1]),
            f"4 × STANDOFF PT-03 AT (±{c.fmt(AXIS)}, ±{c.fmt(AXIS)}),\n"
            f"{c.fmt(CORNER_STANDOFF_INSET_MM)} FROM EACH EDGE, POSITION ±{c.fmt(c.POSITION_TOL_MM)}:\n"
            f"M5 BLIND {c.fmt(sp.BLIND_TAP_THREAD_DEPTH_MM)} DEEP IN THE FRONT PLATE (BACK),\nM5 THRU IN THE BACK PLATE", 88.0)
    sh.text(PLAN_CENTER[0], PLAN_CENTER[1] - hh * PLAN_SCALE + LABEL_DY,
            "FRONT VIEW (FROM THE SENSOR)   SCALE 1:2", size=d.FONT_LABEL, ha="center", weight="bold")

    # ---------------- section A-A (left half) ----------------
    sec = View(sh, SEC_ORIGIN, SEC_SCALE)
    g = SHOWN_GAP_MM
    u_l, u_r = SEC_U_LEFT, SEC_U_RIGHT
    u_axis = -AXIS
    c.region(sec, c.back_plate_piece(u_l, u_r, [u_axis]), crop_u=(u_r,))
    c.break_line(sec, u_r, 0.0, -BACK_T)
    top = g + FRONT_T
    u_edge = -(WINDOW_SIDE / 2) / SECTION_COS  # window front edge in the plane y = 0
    run_s = RUN / SECTION_COS  # bevel run in that plane
    back = c.notched_edge(u_edge - run_s, u_l, g, [u_axis], c.M5_MAJOR_MM, sp.BLIND_TAP_DRILL_DEPTH_MM, up=True)
    poly = [(u_l, top), (u_edge, top), (u_edge, top - c.EDGE_LAND_MM)] + back
    c.region(sec, poly)
    c.region(sec, c.standoff_piece(u_axis, g), other=True)
    sec.center_v(-BACK_T - 4, top + 3, u_axis)
    # dimensions
    sec.dim_h(u_l, u_axis, top, top, 7.0, c.fmt(CORNER_STANDOFF_INSET_MM), outside="right")
    sec.dim_v(u_l, u_l, 0.0, -BACK_T, -8.0, c.fmt(BACK_T))
    sec.dim_v(u_l, u_l, g, top, -8.0, c.fmt(FRONT_T))
    sec.dim_v(u_axis + BODY_D / 2, u_axis + BODY_D / 2, 0.0, g, 10.0, c.fmt(g))
    sh.text(SEC_ORIGIN[0] + u_axis + 22.0, SEC_ORIGIN[1] + g / 2 + 0.8, "STANDOFF PT-03", size=d.FONT_NOTE)
    sh.text(SEC_ORIGIN[0] + u_axis + 22.0, SEC_ORIGIN[1] + g / 2 - 3.6, f"G = {c.fmt(g)} (60 SET: 60)", size=d.FONT_NOTE)
    sh.text(SEC_ORIGIN[0] + (u_l + u_r) / 2, SEC_ORIGIN[1] - BACK_T * SEC_SCALE - 12.0,
            "SECTION A-A (OFFSET)   SCALE 1:1", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(SEC_ORIGIN[0] + (u_l + u_r) / 2, SEC_ORIGIN[1] - BACK_T * SEC_SCALE - 17.5,
            "left half; right half the same. 15 mm set shown", size=d.FONT_NOTE, ha="center")

    # ---------------- 4:1 detail ----------------
    c.draw_edge_detail(sh, DET_ORIGIN, DET_SCALE, FRONT_T, material_side=+1,
                       title="DETAIL B   SCALE 4:1", subtitle="window edge, section at 90° to the edge")

    # ---------------- notes ----------------
    margins = window_hidden_margins()
    notes = [
        f"Front plate PT-05.1 {c.fmt(PLATE_W)} \u00d7 {c.fmt(PLATE_H)} \u00d7 {c.fmt(FRONT_T)}, {c.fmt(WINDOW_SIDE)} square window; "
        f"back plate PT-05.2 {c.fmt(PLATE_W)} \u00d7 {c.fmt(PLATE_H)} \u00d7 {c.fmt(BACK_T)}. Outlines rounded up to whole mm.",
        c.NOTE_KNIFE_EDGE + f" Opening in the back face {c.fmt(round(BACK_OPENING, 2))} square (computed: side + 2 \u00d7 (thickness \u2212 land)).",
        c.NOTE_FINISH + " " + c.NOTE_FLATNESS + " The front face of the back plate is blasted too.",
        c.NOTE_STANDOFF_ENDS,
        f"Standoffs hidden: nearest standoff axis {NEAREST_STANDOFF_MM:.1f} mm from the window; limit at the "
        f"{c.fmt(GAP_LARGE_MM)} mm gap = {c.fmt(GAP_LARGE_MM)} \u00d7 tan {c.fmt(c.HIDE_RAY_ANGLE_DEG)}\u00b0 + "
        f"{c.fmt(c.HIDE_MARGIN_MM)} = {c.hidden_threshold_mm(GAP_LARGE_MM):.1f} mm (margin {margins[GAP_LARGE_MM]:.1f} mm). OK.",
        c.NOTE_SPIGOT,
        f"Section A-A is on y = 0 through the window and crosses its edges at {c.fmt(WINDOW_ROT)}\u00b0 (edge positions "
        f"\u00d7 {1 / SECTION_COS:.4f}); Detail B is the true profile.",
    ]
    bottom = sh.notes_block(NOTES_X, NOTES_TOP, NOTES_W, notes, size=c.NOTE_SIZE)
    assert bottom > d.FRAME_BOTTOM, f"notes overflow the sheet (bottom {bottom:.1f})"
    sh.title_block(TitleInfo("PT-05", "T3b square window, front and back plates", "1 front + 1 back plate",
                             c.MATERIAL_TEXT, c.FINISH_TEXT, "1:2, 1:1, 4:1"))
    path = os.path.join(out_dir, "PT-05_T3b_square_window.png")
    issues = sh.save(path)
    c.report_sheet("PT-05", path, issues)
    return issues


def print_checks() -> None:
    print(f"PT-05 plates {PLATE_W:g} x {PLATE_H:g}; window {WINDOW_SIDE:g} slant {WINDOW_ROT:g} deg; standoff axes at +/-{AXIS:g}")
    print(f"PT-05 countersink run {RUN:.3f} mm; back-face opening side {BACK_OPENING:.3f} mm")
    for g in (GAP_SMALL_MM, GAP_LARGE_MM):
        print(f"PT-05 hidden check, gap {g:g}: nearest standoff axis to the window {NEAREST_STANDOFF_MM:.2f} mm vs "
              f"{c.hidden_threshold_mm(g):.2f} mm (margin {NEAREST_STANDOFF_MM - c.hidden_threshold_mm(g):.2f}); strict "
              f"{c.hidden_threshold_strict_mm(g, FRONT_T):.2f} (margin {NEAREST_STANDOFF_MM - c.hidden_threshold_strict_mm(g, FRONT_T):.2f})")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    print_checks()
    raise SystemExit(1 if build(here) else 0)
