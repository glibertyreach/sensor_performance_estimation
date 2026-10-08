"""Shop drawing PT-04: T3a raised square on its back plate.

Parts on this sheet: the back plate (300 x 300 x 8, spigot pattern in the back
face) and the raised square (160 x 160 x 6, rotated 5 degrees, knife edges
beveled from the back).  Four PT-03 standoffs hold the square above the back
plate on a 60 mm square hidden behind it.

Coordinates
-----------
Plan (front view): target frame, origin at the plate center, x right, y up, seen
from the sensor.  Section A-A: the cutting plane runs along the square's own
u-axis at v = +HIDDEN_POST_SQUARE_MM / 2 (it passes through the axes of two
standoffs and cuts the square edges at right angles).  It is viewed toward +v of
the square's frame, so u runs to the right; z is the height above the back
plate's front face.  Sizes come from the code (feature and plate sizes) or are
named constants below.
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
T3A = TARGET_SET.targets["T3a"]
SQUARE = T3A.features[0]  # the raised square feature
GAP_SMALL_MM = PARAMS.gap_small_mm  # 15 mm set
GAP_LARGE_MM = PARAMS.gap_large_mm  # 60 mm set
PLATE_W, PLATE_H = (c.round_up_mm(v) for v in TARGET_SET.derived["edge_plate_size_mm"])  # back plate, whole mm
SQUARE_SIDE = SQUARE.diameter_mm  # side of the raised square (feature "diameter" of a square is its side)
SQUARE_ROT = SQUARE.rotation_deg  # slant of the square, counterclockwise seen from the sensor
RAISED_SQUARE_THICKNESS_MM = 10.0  # raised square thickness (10, not the 6 mm of a front plate: room for blind holes)
SQUARE_T = RAISED_SQUARE_THICKNESS_MM
HOLE_THREAD_DEPTH_MM = 6.0  # M5 thread depth of the blind standoff holes in the back of the square
HOLE_DRILL_DEPTH_MM = 7.5  # drilled depth of those holes, measured to the tip of the drill point
DRILL_POINT_DEG = 118.0  # included angle of a standard drill point
BACK_T = c.BACK_PLATE_THICKNESS_MM  # back plate thickness (8)
HIDDEN_POST_SQUARE_MM = 60.0  # standoff square (side), in the square's own rotated frame
SHOWN_GAP_MM = GAP_SMALL_MM  # the section shows the 15 mm set; the 60 mm set differs only in standoff length
BODY_D = sp.STANDOFF_BODY_DIAMETER_MM  # standoff body diameter

# Derived dimensions (computed, never typed)
RUN = c.bevel_run(SQUARE_T)  # horizontal run of the back bevel (5.9)
BACK_SIDE = SQUARE_SIDE - 2 * RUN  # side of the square's back face (148.2)
STANDOFF_AXES = [(sx * HIDDEN_POST_SQUARE_MM / 2, sy * HIDDEN_POST_SQUARE_MM / 2)
                 for sx in (-1, 1) for sy in (-1, 1)]  # in the square's frame
assert HIDDEN_POST_SQUARE_MM / 2 + BODY_D / 2 < BACK_SIDE / 2, "standoffs must sit inside the back face"
DRILL_R = c.M5_TAP_DRILL_MM / 2  # tap drill radius
POINT_LEN = DRILL_R / math.tan(math.radians(DRILL_POINT_DEG / 2))  # axial length of the drill point (1.26)
FULL_DIA_DEPTH = HOLE_DRILL_DEPTH_MM - POINT_LEN  # depth to which the drill is at full diameter
FLOOR_MIN = SQUARE_T - HOLE_DRILL_DEPTH_MM  # material left in front of the drill-point tip
assert FLOOR_MIN >= 2.5 - 1e-9, "floor under the blind holes below 2.5 mm"
assert FULL_DIA_DEPTH > HOLE_THREAD_DEPTH_MM, "thread would run into the drill point"
assert sp.STANDOFF_FRONT_STUD_LENGTH_MM <= HOLE_THREAD_DEPTH_MM, "front stud longer than the thread"


def blind_hole_notch(u0: float) -> list[tuple[float, float]]:
    """Points of the blind hole at axis u0, from the left wall at the back face (z = 0 here) to the right wall:
    M5 major diameter to the thread depth, tap drill to the point, cone, and back.  Heights are measured from
    the back face, positive into the square."""
    rm = c.M5_MAJOR_MM / 2
    return [(u0 - rm, 0.0), (u0 - rm, HOLE_THREAD_DEPTH_MM), (u0 - DRILL_R, HOLE_THREAD_DEPTH_MM),
            (u0 - DRILL_R, FULL_DIA_DEPTH), (u0, HOLE_DRILL_DEPTH_MM), (u0 + DRILL_R, FULL_DIA_DEPTH),
            (u0 + DRILL_R, HOLE_THREAD_DEPTH_MM), (u0 + rm, HOLE_THREAD_DEPTH_MM), (u0 + rm, 0.0)]

# ---------------------------------------------------------------------------
# Sheet layout (paper mm)
# ---------------------------------------------------------------------------
PLAN_SCALE = 0.5  # plan at 1:2
PLAN_CENTER = (98.0, 166.0)  # paper position of the plate center
SEC_SCALE = 1.0  # section at 1:1
SEC_ORIGIN = (306.0, 204.0)  # paper position of (u = 0, z = 0): front face of the back plate
SEC_HALF_WIDTH = 98.0  # section is cropped at u = +/- this (break lines)
DET_SCALE = 4.0  # knife-edge detail at 4:1
DET_ORIGIN = (352.0, 160.0)  # paper position of the front-face edge point of the detail
NOTES_X = 12.0  # left edge of the notes
NOTES_TOP = 72.0  # top of the notes block
NOTES_W = 224.0  # width of the notes block
CALL_X = 190.0  # left end of the plan callout texts
CUT_EXT_U = 104.0  # half length of the cutting-plane line, in the square's frame
ANGLE_ARC_PAPER = 58.0  # radius of the 5 degree dimension arc, paper mm
DIM_TOP = 7.0  # overall width dimension above the plan (paper)
DIM_LEFT = -8.0  # overall height dimension left of the plan (paper)
LABEL_DY = -9.0  # plan label below the plate (paper)


def build(out_dir: str) -> list[str]:
    """Draw PT-04 and return the layout problems found."""
    sh = Sheet("PT-04")
    sh.border()
    plan = View(sh, PLAN_CENTER, PLAN_SCALE)
    hw, hh = PLATE_W / 2, PLATE_H / 2
    rot = SQUARE_ROT

    def sq(u: float, v: float) -> tuple[float, float]:
        """Square-frame point -> target frame."""
        return c.rotate(u, v, rot)

    # ---------------- plan view (front view) ----------------
    plan.polyline([(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)], "outline", closed=True)
    h = SQUARE_SIDE / 2
    plan.polyline([sq(-h, -h), sq(h, -h), sq(h, h), sq(-h, h)], "outline", closed=True)
    hb = BACK_SIDE / 2
    plan.polyline([sq(-hb, -hb), sq(hb, -hb), sq(hb, hb), sq(-hb, hb)], "hidden", closed=True)
    for (su, sv) in STANDOFF_AXES:
        p = sq(su, sv)
        plan.circle(p, BODY_D / 2, "hidden")
        plan.center_cross(p, BODY_D / 2)
    c.plan_spigot_pattern(plan)
    plan.center_h(-hw - 6, hw + 6, 0.0)
    plan.center_v(-hh - 6, hh + 6, 0.0)
    # cutting plane A-A along the square's u-axis at v = +60/2 (through two standoff axes)
    v_cut = HIDDEN_POST_SQUARE_MM / 2
    sight = c.rotate(0.0, 1.0, rot)
    plan.cutting_plane([sq(-CUT_EXT_U, v_cut), sq(CUT_EXT_U, v_cut)], sight=sight, label="A")
    # overall size
    plan.dim_h(-hw, hw, hh, hh, DIM_TOP, f"{c.fmt(PLATE_W)}")
    plan.dim_v(-hw, -hw, -hh, hh, DIM_LEFT, f"{c.fmt(PLATE_H)}")
    # slant of the square: angle between the horizontal and the lower edge
    corner = sq(-h, -h)
    ref_len = ANGLE_ARC_PAPER / PLAN_SCALE + 6.0
    plan.line(corner, (corner[0] + ref_len, corner[1]), "thin")
    plan.dim_angle(corner, 0.0, rot, ANGLE_ARC_PAPER, f"{c.fmt(rot)}°", text_dx=7.0, text_dy=-1.5)

    # callouts: leader to the feature, text in the right-hand column
    def callout(target, text, slot_y, size=d.FONT_NOTE):
        t = plan.P(*target)
        return plan.leader(target, text, CALL_X - t[0] - d.LEADER_SHOULDER - d.LEADER_TEXT_GAP, slot_y - t[1],
                           size=size)

    arrow = c.orientation_arrow()
    callout(arrow[3], f"ORIENTATION ARROW, BACK FACE, POINTS {sp.SPIGOT_DOWEL_DIRECTION}:\n"
                      f"{c.fmt(sp.ORIENTATION_MARK_WIDTH_MM)} WIDE \u00d7 {c.fmt(sp.ORIENTATION_MARK_DEPTH_MM)} DEEP", 172.0)
    callout(sq(h, 0.0), f"{c.fmt(SQUARE_SIDE)} \u00d7 {c.fmt(SQUARE_SIDE)} \u00d7 {c.fmt(SQUARE_T)} RAISED SQUARE,\n"
                        f"SLANT {c.fmt(rot)}\u00b0 COUNTERCLOCKWISE", 155.0)
    sp_pts = c.spigot_hole_points()
    callout((sp_pts[3][0] + c.M5_MAJOR_MM / 2, sp_pts[3][1]),
            f"4 \u00d7 {sp.SPIGOT_SCREW} THRU IN THE BACK PLATE (SPIGOT PT-02),\n"
            f"\u00d8{c.fmt(sp.SPIGOT_PCD_MM)} PCD AT {', '.join(c.fmt(a) for a in sp.SPIGOT_HOLE_ANGLES_DEG)}\u00b0 FROM +x,\n"
            f"CENTERED ON THE PLATE; \u00d8{c.fmt(sp.SPIGOT_FLANGE_DIAMETER_MM)} FLANGE (PHANTOM)", 134.0)
    post_edge = sq(*STANDOFF_AXES[3])  # lower right standoff in the target frame
    callout((post_edge[0] + BODY_D / 2, post_edge[1]),
            f"4 \u00d7 STANDOFF PT-03 ON A {c.fmt(HIDDEN_POST_SQUARE_MM)} SQUARE\n"
            f"(\u00b1{c.fmt(HIDDEN_POST_SQUARE_MM / 2)}, \u00b1{c.fmt(HIDDEN_POST_SQUARE_MM / 2)} IN THE SQUARE'S FRAME)\n"
            f"M5 BLIND {c.fmt(HOLE_THREAD_DEPTH_MM)} DEEP IN THE SQUARE (BACK),\n"
            f"M5 THRU IN THE BACK PLATE; POSITION \u00b1{c.fmt(c.POSITION_TOL_MM)}", 112.0)
    callout(sq(hb, -hb * 0.55), f"BACK FACE {c.fmt(round(BACK_SIDE, 2))} \u00d7 {c.fmt(round(BACK_SIDE, 2))} (HIDDEN):\n"
                                f"4 EDGES BEVELED {c.fmt(c.BEVEL_DEG)}\u00b0 FROM THE BACK,\nLAND {c.LAND_TEXT}", 90.0)
    sh.text(PLAN_CENTER[0], PLAN_CENTER[1] - hh * PLAN_SCALE + LABEL_DY,
            "FRONT VIEW (FROM THE SENSOR)   SCALE 1:2", size=d.FONT_LABEL, ha="center", weight="bold")

    # ---------------- section A-A ----------------
    sec = View(sh, SEC_ORIGIN, SEC_SCALE)
    u_l, u_r = -SEC_HALF_WIDTH, SEC_HALF_WIDTH
    g = SHOWN_GAP_MM
    hole_us = [-HIDDEN_POST_SQUARE_MM / 2, HIDDEN_POST_SQUARE_MM / 2]
    # back plate
    c.region(sec, c.back_plate_piece(u_l, u_r, hole_us), crop_u=(u_l, u_r))
    # raised square: front face at z = g + SQUARE_T, back face at z = g, bevel at both ends
    top = g + SQUARE_T
    front = [(-h, top), (h, top), (h, top - c.EDGE_LAND_MM), (hb, g)]
    back = [(hb, g)]
    for u0 in sorted(hole_us, reverse=True):  # right to left along the back face
        back += [(u, g + z) for (u, z) in blind_hole_notch(u0)[::-1]]
    sq_poly = front + back[1:] + [(-hb, g), (-h, top - c.EDGE_LAND_MM)]
    c.region(sec, sq_poly)
    # standoffs
    for u0 in hole_us:
        c.region(sec, c.standoff_piece(u0, g), other=True)
        sec.center_v(-BACK_T - 4, top + 3, u0)
    c.break_line(sec, u_l, 0.0, -BACK_T)
    c.break_line(sec, u_r, 0.0, -BACK_T)
    # dimensions
    sec.dim_h(-HIDDEN_POST_SQUARE_MM / 2, HIDDEN_POST_SQUARE_MM / 2, top, top, 7.0, c.fmt(HIDDEN_POST_SQUARE_MM))
    sec.dim_h(-h, h, top, top, 16.0, c.fmt(SQUARE_SIDE), base_v=top)
    sec.dim_v(u_l, u_l, 0.0, -BACK_T, -8.0, c.fmt(BACK_T))
    sec.dim_v(-HIDDEN_POST_SQUARE_MM / 2 - BODY_D / 2, -HIDDEN_POST_SQUARE_MM / 2 - BODY_D / 2, 0.0, g, -9.0,
              c.fmt(g))
    sec.dim_v(h, hb, top, g, 10.0, c.fmt(SQUARE_T))
    sh.text(SEC_ORIGIN[0] + 38.0, SEC_ORIGIN[1] + g / 2 * SEC_SCALE - 1.2, "STANDOFF PT-03", size=d.FONT_NOTE)
    sh.text(SEC_ORIGIN[0] + 38.0, SEC_ORIGIN[1] + g / 2 * SEC_SCALE - 5.2, f"G = {c.fmt(g)} (60 SET: 60)", size=d.FONT_NOTE)
    sh.text(SEC_ORIGIN[0], SEC_ORIGIN[1] - BACK_T * SEC_SCALE - 12.0,
            "SECTION A-A   SCALE 1:1", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(SEC_ORIGIN[0], SEC_ORIGIN[1] - BACK_T * SEC_SCALE - 17.5,
            "back plate, 15 mm set shown; plate and square cropped at both ends", size=d.FONT_NOTE, ha="center")

    # ---------------- 4:1 knife-edge detail ----------------
    c.draw_edge_detail(sh, DET_ORIGIN, DET_SCALE, SQUARE_T, material_side=-1,
                       title="DETAIL B   SCALE 4:1", subtitle="knife edge of the square, section at 90° to the edge")

    # ---------------- notes ----------------
    notes = [
        f"Back plate PT-04.1 {c.fmt(PLATE_W)} \u00d7 {c.fmt(PLATE_H)} \u00d7 {c.fmt(BACK_T)}; raised square PT-04.2 "
        f"{c.fmt(SQUARE_SIDE)} \u00d7 {c.fmt(SQUARE_SIDE)} \u00d7 {c.fmt(SQUARE_T)}, slanted {c.fmt(SQUARE_ROT)}\u00b0 about the plate "
        f"center. Plate outline rounded up to whole mm.",
        c.NOTE_KNIFE_EDGE + f" Back face of the square {c.fmt(round(BACK_SIDE, 2))} square (computed).",
        c.NOTE_FINISH + " " + c.NOTE_FLATNESS + " Do not machine the front face or the land after blasting.",
        f"Standoffs PT-03, 15 mm set or 60 mm set; both sets delivered. Square: {sp.STANDOFF_STUD} blind holes, thread "
        f"{c.fmt(HOLE_THREAD_DEPTH_MM)} deep, tap drill \u00d8{c.M5_TAP_DRILL_MM:g} drilled {c.fmt(HOLE_DRILL_DEPTH_MM)} deep to the tip of a "
        f"{c.fmt(DRILL_POINT_DEG)}\u00b0 point (full diameter to {FULL_DIA_DEPTH:.2f}); floor under the tip {FLOOR_MIN:.1f} mm. The "
        f"{c.fmt(sp.STANDOFF_FRONT_STUD_LENGTH_MM)} mm front stud seats {c.fmt(HOLE_THREAD_DEPTH_MM - sp.STANDOFF_FRONT_STUD_LENGTH_MM)} "
        f"mm short of the thread bottom. Back plate: M5 through-tapped (rotated {c.fmt(SQUARE_ROT)}\u00b0 with the square) "
        f"for the {c.fmt(sp.STANDOFF_STUD_LENGTH_MM)} mm studs, not proud of the back face. Hole position \u00b1{c.fmt(c.POSITION_TOL_MM)}.",
        c.NOTE_SPIGOT,
    ]
    bottom = sh.notes_block(NOTES_X, NOTES_TOP, NOTES_W, notes, size=c.NOTE_SIZE)
    assert bottom > d.FRAME_BOTTOM, f"notes overflow the sheet (bottom {bottom:.1f})"

    sh.title_block(TitleInfo("PT-04", "T3a raised square and back plate", "1 plate + 1 square",
                             c.MATERIAL_TEXT, c.FINISH_TEXT, "1:2, 1:1, 4:1"))
    path = os.path.join(out_dir, "PT-04_T3a_raised_square.png")
    issues = sh.save(path)
    c.report_sheet("PT-04", path, issues)
    return issues


def print_checks() -> None:
    """Print the computed values that the drawing uses."""
    print(f"PT-04 plate {PLATE_W:g} x {PLATE_H:g} x {BACK_T:g}; square {SQUARE_SIDE:g} x {SQUARE_T:g}, slant {SQUARE_ROT:g} deg")
    print(f"PT-04 bevel run {RUN:.3f} mm, back face side {BACK_SIDE:.3f} mm")
    for g in (GAP_SMALL_MM, GAP_LARGE_MM):
        axis_to_edge = SQUARE_SIDE / 2 - HIDDEN_POST_SQUARE_MM / 2  # standoff axis to the nearest front-face edge
        print(f"PT-04 hidden check, gap {g:g}: axis-to-edge {axis_to_edge:.2f} mm vs gap x tan30 + 5 = "
              f"{c.hidden_threshold_mm(g):.2f} mm (margin {axis_to_edge - c.hidden_threshold_mm(g):.2f}); "
              f"strict {c.hidden_threshold_strict_mm(g, SQUARE_T):.2f} (margin "
              f"{axis_to_edge - c.hidden_threshold_strict_mm(g, SQUARE_T):.2f})")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    print_checks()
    raise SystemExit(1 if build(here) else 0)
