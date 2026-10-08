"""Shop drawing PT-07: T5 cutout plate with its edge bracket.

Parts on this sheet
-------------------
PT-07.1  front plate 336 x 198 x 6 with three through cutouts (countersunk from the back), four
         M5 through-tapped standoff holes and three M5 through-tapped bracket-screw holes;
PT-07.2  back plate 336 x 198 x 8, plain, removable (open-background captures), four M5
         through-tapped standoff holes; it does NOT carry the spigot;
PT-07.3  edge bracket: aluminum bar 40 x 198 x 8 screwed to the BACK face of the front plate along its
         left edge, with a 100 mm extension (90 wide) beyond the edge that carries the spigot pattern.

Coordinates
-----------
Plan (front view): target frame, origin at the plate center, x right, y up, seen from the sensor.
Section A-A is an offset section (plane y = -BRACKET_SCREW_Y through the bracket screw and the left
standoff, then y = y of the largest cutout through its center), viewed toward +y so that u = x; z is
the height above the front face of the back plate.  The section is cropped (break lines) to the
left edge and to the largest cutout.  Site positions and diameters come from ``ts.targets["T5"]``.
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
T5 = TARGET_SET.targets["T5"]
GAP_SMALL_MM = PARAMS.gap_small_mm
GAP_LARGE_MM = PARAMS.gap_large_mm
PLATE_W = c.round_up_mm(2 * T5.half_width_mm)  # 335.94 -> 336
PLATE_H = c.round_up_mm(2 * T5.half_height_mm)  # 197.63 -> 198
FRONT_T = c.FRONT_PLATE_THICKNESS_MM  # front plate (6)
BACK_T = c.BACK_PLATE_THICKNESS_MM  # back plate (8)
BODY_D = sp.STANDOFF_BODY_DIAMETER_MM
CORNER_STANDOFF_INSET_MM = 12.0  # right-hand standoffs: distance of the axes from the right and top/bottom edges
LEFT_STANDOFF_X_MM = -120.0  # left-hand standoffs: x of their axes (clear of the bracket)
CUTOUTS = sorted((f for f in T5.features if f.kind == "cutout"), key=lambda f: f.level_index)
BLANKS = sorted((f for f in T5.features if f.kind == "blank"), key=lambda f: f.site_id)
assert len(CUTOUTS) == 3 and len(BLANKS) == 3, "T5 sites changed: revisit the drawing"
LARGEST = CUTOUTS[-1]  # the section goes through this cutout
SHOWN_GAP_MM = GAP_SMALL_MM  # G of the section: the 15 mm set
BODY_SHOWN_MM = c.standoff_body_length(SHOWN_GAP_MM, FRONT_T)  # standoff body length = space between the plates in the section (G - 6 = 9)

# Edge bracket
BRACKET_WIDTH_MM = 40.0  # bar width (x), against the left edge of the front plate
BRACKET_THICKNESS_MM = 8.0  # bar thickness: it lies in the gap behind the front plate
BRACKET_LENGTH_MM = PLATE_H  # bar length (y) = the plate height
BRACKET_EXTENSION_MM = 100.0  # extension beyond the left edge, carries the spigot
BRACKET_EXTENSION_WIDTH_MM = 90.0  # width (y) of the extension, so the 80 mm flange fits
BRACKET_SCREW_COUNT = 3  # M5 screws into the front plate
BRACKET_SCREW_Y_MM = PLATE_H / 2 - CORNER_STANDOFF_INSET_MM  # screws at y = -87, 0, +87
SCREW_HEAD_D = 8.5  # DIN 7984 M5 head diameter
SCREW_HEAD_H = 3.0  # DIN 7984 M5 head height
SCREW_CLEAR_D = 5.5  # clearance hole in the bracket
SCREW_CBORE_D = 9.5  # counterbore diameter in the bracket (opens on its back face)
SCREW_CBORE_DEPTH = 4.5  # counterbore depth
SCREW_LENGTHS = (6, 8, 10, 12, 16, 20)  # DIN 7984 M5 stock lengths
ARROW_BAR_Y_MM = 10.0  # the orientation arrow on the bracket's back face starts at this y (on the bar)
ARROW_TAIL_DEFAULT_MM = sp.SPIGOT_FLANGE_DIAMETER_MM / 2 + sp.ORIENTATION_MARK_CLEARANCE_MM  # tail y of c.orientation_arrow()

# Derived geometry (computed, never typed)
PLATE_X0 = -PLATE_W / 2  # left edge of the plate (-168)
BAR_X0, BAR_X1 = PLATE_X0, PLATE_X0 + BRACKET_WIDTH_MM  # bar outer face flush with the plate's left edge
BAR_CX = (BAR_X0 + BAR_X1) / 2  # screw column (-148)
EXT_X0 = PLATE_X0 - BRACKET_EXTENSION_MM  # outer end of the extension (-268)
SPIGOT_CENTER = (PLATE_X0 - BRACKET_EXTENSION_MM / 2, 0.0)  # (-218, 0)
BRACKET_WEB = BRACKET_THICKNESS_MM - SCREW_CBORE_DEPTH  # bracket material under the screw head (3.5)
SCREW_MAX_LEN = BRACKET_WEB + FRONT_T - c.FRONT_STUD_RECESS_MM  # longest screw whose tip stays 0.5 below the front face
SCREW_LEN = max(l for l in SCREW_LENGTHS if l <= SCREW_MAX_LEN + 1e-9)  # standard length that does not protrude
SCREW_ENGAGEMENT = SCREW_LEN - BRACKET_WEB  # thread length in the front plate
SCREW_TIP_BELOW_FRONT = FRONT_T - SCREW_ENGAGEMENT  # tip below the front face (margin)
SCREW_HEAD_RECESS = SCREW_CBORE_DEPTH - SCREW_HEAD_H  # head top below the bracket's back face
assert SCREW_TIP_BELOW_FRONT >= c.FRONT_STUD_RECESS_MM - 1e-9, "screw tip would protrude the front face"
assert SCREW_HEAD_D < SCREW_CBORE_D < BRACKET_WIDTH_MM, "counterbore does not fit the bar"
assert BRACKET_EXTENSION_WIDTH_MM > sp.SPIGOT_FLANGE_DIAMETER_MM, "flange does not fit the extension"
assert BRACKET_THICKNESS_MM < BODY_SHOWN_MM, "bracket must lie between the plates at the small gap"
BRACKET_CLEARANCE_MM = BODY_SHOWN_MM - BRACKET_THICKNESS_MM  # bracket back face to the back plate (1 mm)
STANDOFF_Y_MM = PLATE_H / 2 - CORNER_STANDOFF_INSET_MM  # |y| of all standoff axes (87)
STANDOFFS = [(x, sy * STANDOFF_Y_MM) for x in (PLATE_W / 2 - CORNER_STANDOFF_INSET_MM, LEFT_STANDOFF_X_MM) for sy in (-1, 1)]
SCREW_POINTS = [(BAR_CX, k * BRACKET_SCREW_Y_MM) for k in (-1, 0, 1)]
ARROW_PTS = [(BAR_CX + px, py - ARROW_TAIL_DEFAULT_MM + ARROW_BAR_Y_MM) for (px, py) in c.orientation_arrow()]  # engraved arrow, back face of the bar
assert LEFT_STANDOFF_X_MM - BODY_D / 2 > BAR_X1, "left standoffs must clear the bracket"


def nearest_edge_distance(p: tuple[float, float]) -> float:
    """Distance from a standoff axis to the nearest cutout edge."""
    return min(math.hypot(p[0] - f.x_mm, p[1] - f.y_mm) - f.diameter_mm / 2 for f in CUTOUTS)


NEAREST_CUTOUT_EDGE_MM = min(nearest_edge_distance(p) for p in STANDOFFS)
BRACKET_TO_CUTOUT_MM = min(f.x_mm - f.diameter_mm / 2 - BAR_X1 for f in CUTOUTS)  # bar inner edge to the nearest cutout edge
assert NEAREST_CUTOUT_EDGE_MM > c.hidden_threshold_mm(GAP_LARGE_MM), "standoffs would be visible through a cutout"


def csk_back_diameter(f) -> float:
    """Countersink diameter in the back face of the front plate: D + 2 x (thickness - land) at 45 degrees."""
    return f.diameter_mm + 2 * c.bevel_run(FRONT_T)


# ---------------------------------------------------------------------------
# Sheet layout (paper mm)
# ---------------------------------------------------------------------------
PLAN_SCALE = 0.5  # plan at 1:2
PLAN_ORIGIN = (156.0, 196.0)  # paper position of the plate center
SEC_ORIGIN_Y = 108.0  # paper y of z = 0 in the section
SEC_SEGMENTS = (  # (u start, u end, paper x of the start): the two cropped pieces of the section
    (PLATE_X0, PLATE_X0 + 58.0, 14.0),
    (LARGEST.x_mm - 42.0, LARGEST.x_mm + 42.0, 78.0),
)
DET_ORIGIN = (216.0, 128.0)  # paper position of the front-face edge point of the 4:1 detail
DET_SCALE = 4.0
BR_ORIGIN = (292.0, 160.0)  # paper position of the bracket plan point (x = bracket center, y = 0)
BR_SCALE = 0.5  # bracket plan at 1:2
BR_CX = (EXT_X0 + BAR_X1) / 2  # model x of the bracket plan center
SIDE_ORIGIN = (328.0, 84.0)  # paper position of (x = BR_CX, front face) of the side view
SIDE_SCALE = 1.0  # side view at 1:1
TABLE_X, TABLE_TOP = 248.0, 256.0
NOTES_X, NOTES_TOP, NOTES_W = 12.0, 82.0, 224.0
CALL_X = 352.0  # left end of the bracket callout texts
LABEL_INSIDE = ("blank_02",)  # site ids placed inside their circle (the hidden bar edge crosses the space above)
BOX_X, BOX_TOP, BOX_W = 346.0, 150.0, 62.0  # boxed note (hidden-standoff check) right of the detail


def site_rows() -> list[list[str]]:
    rows = [["Site", "Kind", "x", "y", "Front Ø ±0.02", "Back Ø (CSK)"]]
    for f in CUTOUTS:
        rows.append([f.site_id, "cutout", f"{f.x_mm:+.2f}", f"{f.y_mm:+.2f}", c.fmt2(f.diameter_mm), c.fmt2(csk_back_diameter(f))])
    for f in BLANKS:
        rows.append([f.site_id, "blank", f"{f.x_mm:+.2f}", f"{f.y_mm:+.2f}", f"{c.fmt2(f.diameter_mm)} (patch)", "none"])
    return rows


def build(out_dir: str) -> list[str]:
    sh = Sheet("PT-07")
    sh.border()
    plan = View(sh, PLAN_ORIGIN, PLAN_SCALE)
    hw, hh = PLATE_W / 2, PLATE_H / 2
    plan.polyline([(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)], "outline", closed=True)
    for f in CUTOUTS:
        plan.circle((f.x_mm, f.y_mm), f.diameter_mm / 2, "outline")
        plan.circle((f.x_mm, f.y_mm), csk_back_diameter(f) / 2, "hidden")
        plan.center_cross((f.x_mm, f.y_mm), f.diameter_mm / 2)
    for f in BLANKS:
        plan.circle((f.x_mm, f.y_mm), f.diameter_mm / 2, "phantom")
    for f in CUTOUTS + BLANKS:
        r = (csk_back_diameter(f) if f.kind == "cutout" else f.diameter_mm) / 2
        if f.site_id in LABEL_INSIDE:
            pt = plan.P(BAR_X1 + 4.0, f.y_mm)  # right of the hidden bar edge, above the cutting-plane line
            sh.text(pt[0], pt[1] + 3.0, f.site_id, size=d.FONT_MIN, ha="left")
        else:
            pt = plan.P(f.x_mm, f.y_mm + r)
            sh.text(pt[0], pt[1] + 1.8 + max(d.CENTER_CROSS_EXT - r * PLAN_SCALE, 0.0), f.site_id, size=d.FONT_MIN, ha="center")
    for p in STANDOFFS:
        plan.circle(p, BODY_D / 2, "hidden")  # standoff body behind the front plate
        c.plan_tapped_hole(plan, p, visible=True)
    for p in SCREW_POINTS:
        plan.circle(p, SCREW_CBORE_D / 2, "hidden")  # counterbore on the bracket's back face
        c.plan_tapped_hole(plan, p, visible=True)  # M5 through-tapped hole in the front plate
    # bracket: the bar is hidden behind the plate, the extension is in plain view
    plan.polyline([(BAR_X1, hh), (BAR_X0, hh), (BAR_X0, -hh), (BAR_X1, -hh), (BAR_X1, hh)], "hidden")
    plan.polyline(ARROW_PTS, "hidden", closed=True)
    plan.polyline([(BAR_X0, BRACKET_EXTENSION_WIDTH_MM / 2), (EXT_X0, BRACKET_EXTENSION_WIDTH_MM / 2),
                   (EXT_X0, -BRACKET_EXTENSION_WIDTH_MM / 2), (BAR_X0, -BRACKET_EXTENSION_WIDTH_MM / 2)], "outline")
    plan.circle(SPIGOT_CENTER, sp.SPIGOT_FLANGE_DIAMETER_MM / 2, "phantom")
    plan.circle(SPIGOT_CENTER, sp.SPIGOT_PCD_MM / 2, "center")
    for p in c.spigot_hole_points(SPIGOT_CENTER):
        c.plan_tapped_hole(plan, p, visible=True)
    plan.center_h(EXT_X0 - 6, hw + 6, 0.0)
    plan.center_v(hh, hh + 5, 0.0)  # center marks at the top and bottom edges only
    plan.center_v(-hh - 5, -hh, 0.0)
    plan.center_v(-BRACKET_EXTENSION_WIDTH_MM / 2 - 5, BRACKET_EXTENSION_WIDTH_MM / 2 + 5, SPIGOT_CENTER[0])
    # cutting plane A-A: bracket screw + left standoff, then the center of the largest cutout
    yc = LARGEST.y_mm
    jog_x = LEFT_STANDOFF_X_MM + 20.0
    plan.cutting_plane([(PLATE_X0 - 8.0, -BRACKET_SCREW_Y_MM), (jog_x, -BRACKET_SCREW_Y_MM), (jog_x, yc),
                        (LARGEST.x_mm + LARGEST.diameter_mm / 2 + 8.0, yc)], sight=(0.0, 1.0), label="A")
    # overall dimensions
    plan.dim_h(EXT_X0, PLATE_X0, hh, hh, 7.0, c.fmt(BRACKET_EXTENSION_MM))
    plan.dim_h(PLATE_X0, hw, hh, hh, 7.0, c.fmt(PLATE_W))
    plan.dim_v(EXT_X0, EXT_X0, -hh, hh, -8.0, c.fmt(PLATE_H))
    sh.text(24.0, 238.0, "FRONT VIEW", size=d.FONT_LABEL, weight="bold")
    sh.text(24.0, 232.5, "SCALE 1:2", size=d.FONT_LABEL, weight="bold")

    def callout(target, text, tx, ty):
        t = plan.P(*target)
        return plan.leader(target, text, tx - t[0] - d.LEADER_SHOULDER - d.LEADER_TEXT_GAP, ty - t[1], size=d.FONT_MIN)

    b1 = BLANKS[1]
    tx, ty = plan.P(60.0, -85.0)  # text in the free corner below blank_01, between cutout_02 and the standoff
    sh.text(tx, ty + 1.2, "BLANK SITE: NO FEATURE,", size=d.FONT_MIN)
    sh.text(tx, ty - 3.0, "KEEP FREE OF MARKS", size=d.FONT_MIN)
    bt = plan.P(b1.x_mm - b1.diameter_mm / 2 * 0.5, b1.y_mm - b1.diameter_mm / 2 * math.sqrt(0.75))
    sh.line([(tx + 16.0, ty + 6.0), bt], "thin")
    sh.arrow(bt, (bt[0] - tx - 16.0, bt[1] - ty - 6.0))

    # ---------------- section A-A (offset, cropped) ----------------
    g = BODY_SHOWN_MM  # back face of the front plate above the back plate = standoff body length
    top = g + FRONT_T
    (a0, a1, ax), (b0, b1, bx) = SEC_SEGMENTS
    s1 = View(sh, (ax - a0, SEC_ORIGIN_Y), 1.0)
    s2 = View(sh, (bx - b0, SEC_ORIGIN_Y), 1.0)
    # ---- piece 1: left edge, bracket, screw, standoff ----
    ux = LEFT_STANDOFF_X_MM
    c.region(s1, c.back_plate_piece(a0, a1, [ux]), crop_u=(a1,))
    hole_r = c.M5_MAJOR_MM / 2
    fp = [(a0, BAR_CX - hole_r), (BAR_CX + hole_r, ux - hole_r), (ux + hole_r, a1)]
    for lo, hi in fp:
        c.region(s1, [(lo, top), (hi, top), (hi, g), (lo, g)], crop_u=(a1,))
    z_bar0 = g - BRACKET_THICKNESS_MM
    cb, cl = SCREW_CBORE_D / 2, SCREW_CLEAR_D / 2
    z_cb = z_bar0 + SCREW_CBORE_DEPTH
    c.region(s1, [(BAR_X0, z_bar0), (BAR_CX - cb, z_bar0), (BAR_CX - cb, z_cb), (BAR_CX - cl, z_cb), (BAR_CX - cl, g), (BAR_X0, g)])
    c.region(s1, [(BAR_CX + cb, z_bar0), (BAR_X1, z_bar0), (BAR_X1, g), (BAR_CX + cl, g), (BAR_CX + cl, z_cb), (BAR_CX + cb, z_cb)])
    hr = SCREW_HEAD_D / 2
    zt = z_cb + SCREW_LEN
    c.region(s1, [(BAR_CX - hr, z_cb - SCREW_HEAD_H), (BAR_CX + hr, z_cb - SCREW_HEAD_H), (BAR_CX + hr, z_cb),
                  (BAR_CX + hole_r, z_cb), (BAR_CX + hole_r, zt), (BAR_CX - hole_r, zt), (BAR_CX - hole_r, z_cb),
                  (BAR_CX - hr, z_cb)], other=True)
    c.region(s1, c.standoff_piece(ux, g), other=True)
    c.break_line(s1, a1, 0.0, -BACK_T)
    c.break_line(s1, a1, g, top)
    for u in (BAR_CX, ux):
        s1.center_v(-BACK_T - 4, top + 3, u)
    s1.dim_h(PLATE_X0, BAR_CX, top, top, 7.0, c.fmt(BAR_CX - PLATE_X0))
    s1.dim_h(BAR_CX, ux, top, top, 7.0, c.fmt(ux - BAR_CX))
    # ---- piece 2: the largest cutout ----
    u_ef_l, u_ef_r = LARGEST.x_mm - LARGEST.diameter_mm / 2, LARGEST.x_mm + LARGEST.diameter_mm / 2
    run = c.bevel_run(FRONT_T)
    c.region(s2, [(b0, -BACK_T), (b1, -BACK_T), (b1, 0.0), (b0, 0.0)], crop_u=(b0, b1))
    c.region(s2, [(b0, top), (u_ef_l, top), (u_ef_l, top - c.EDGE_LAND_MM), (u_ef_l - run, g), (b0, g)], crop_u=(b0,))
    c.region(s2, [(b1, top), (u_ef_r, top), (u_ef_r, top - c.EDGE_LAND_MM), (u_ef_r + run, g), (b1, g)], crop_u=(b1,))
    for u in (b0, b1):
        c.break_line(s2, u, 0.0, -BACK_T)
        c.break_line(s2, u, g, top)
    s2.center_v(-BACK_T - 4, top + 3, LARGEST.x_mm)
    s2.dim_h(u_ef_l, u_ef_r, top, top, 7.0, f"Ø{c.fmt2(LARGEST.diameter_mm)} ±{c.fmt(c.SITE_DIAMETER_TOL_MM)}")
    s2.dim_h(u_ef_l - run, u_ef_r + run, g, g, -8.0, f"Ø{c.fmt2(csk_back_diameter(LARGEST))}", text_pos=0.25)
    s2.dim_v(b1, b1, -BACK_T, 0.0, 8.0, c.fmt(BACK_T), outside="down")
    s2.dim_v(b1, b1, g, top, 8.0, c.fmt(FRONT_T), outside="down")
    s2.dim_v(b1, b1, 0.0, g, 17.0, c.fmt(g))
    s2.dim_v(b1, b1, 0.0, top, 26.0, c.fmt(SHOWN_GAP_MM), outside="down")  # G: front face of the front plate to the back plate
    sh.text((ax + bx + (b1 - b0)) / 2, SEC_ORIGIN_Y - BACK_T - 11.0, "SECTION A-A (OFFSET)   SCALE 1:1",
            size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text((ax + bx + (b1 - b0)) / 2, SEC_ORIGIN_Y - BACK_T - 16.5, f"G = {c.fmt(SHOWN_GAP_MM)} shown (body {c.fmt(g)}); cropped", size=d.FONT_NOTE, ha="center")

    # ---------------- 4:1 detail of the countersink ----------------
    c.draw_edge_detail(sh, DET_ORIGIN, DET_SCALE, FRONT_T, material_side=+1, title="DETAIL B   SCALE 4:1",
                       subtitle="cutout edge, 90° to the edge", face_label="FRONT FACE",
                       land_label=f"LAND\n{c.LAND_TEXT}", extra_mm=0.5, land_dy=12.0)

    # ---------------- table ----------------
    sh.table(TABLE_X, TABLE_TOP, [24.0, 14.0, 21.0, 21.0, 32.0, 30.0], site_rows(),
             f"SITES (target frame); POSITION ±{c.fmt(c.POSITION_TOL_MM)}", size=d.FONT_MIN, row_h=4.8)

    # ---------------- bracket plan (front view) and side view ----------------
    bp = View(sh, BR_ORIGIN, BR_SCALE)

    def B(x, y):
        return (x - BR_CX, y)

    def bpoly(pts, style, closed=True):
        bp.polyline([B(*p) for p in pts], style, closed=closed)

    bpoly([(BAR_X1, hh), (BAR_X0, hh), (BAR_X0, BRACKET_EXTENSION_WIDTH_MM / 2), (EXT_X0, BRACKET_EXTENSION_WIDTH_MM / 2),
           (EXT_X0, -BRACKET_EXTENSION_WIDTH_MM / 2), (BAR_X0, -BRACKET_EXTENSION_WIDTH_MM / 2), (BAR_X0, -hh), (BAR_X1, -hh)],
          "outline")
    for p in SCREW_POINTS:
        bp.circle(B(*p), SCREW_CLEAR_D / 2, "outline")
        bp.circle(B(*p), SCREW_CBORE_D / 2, "hidden")
        bp.center_cross(B(*p), SCREW_CBORE_D / 2)
    bp.circle(B(*SPIGOT_CENTER), sp.SPIGOT_FLANGE_DIAMETER_MM / 2, "phantom")
    bp.circle(B(*SPIGOT_CENTER), sp.SPIGOT_PCD_MM / 2, "center")
    for p in c.spigot_hole_points(SPIGOT_CENTER):
        bp.circle(B(*p), c.M5_MINOR_MM / 2, "outline")
        bp.circle(B(*p), c.M5_MAJOR_MM / 2, "thin", 0.0, 270.0)
        bp.center_cross(B(*p), c.M5_MAJOR_MM / 2)
    bpoly(ARROW_PTS, "hidden")
    bp.line(B(EXT_X0 - 4, 0.0), B(BAR_X1 + 4, 0.0), "center")
    bp.line(B(SPIGOT_CENTER[0], -hh - 4), B(SPIGOT_CENTER[0], hh + 4), "center")
    bp.line(B(BAR_CX, -hh - 4), B(BAR_CX, hh + 4), "center")
    bp.dim_v(EXT_X0 - BR_CX, EXT_X0 - BR_CX, -BRACKET_EXTENSION_WIDTH_MM / 2, BRACKET_EXTENSION_WIDTH_MM / 2, -8.0,
             c.fmt(BRACKET_EXTENSION_WIDTH_MM))
    bp.dim_v(BAR_X1 - BR_CX, BAR_X1 - BR_CX, -hh, hh, 8.0, c.fmt(BRACKET_LENGTH_MM))
    bp.dim_h(EXT_X0 - BR_CX, BAR_X0 - BR_CX, -BRACKET_EXTENSION_WIDTH_MM / 2, -BRACKET_EXTENSION_WIDTH_MM / 2, -8.0,
             c.fmt(BRACKET_EXTENSION_MM), base_v=-hh)
    bp.dim_h(BAR_X0 - BR_CX, BAR_X1 - BR_CX, -hh, -hh, -8.0, c.fmt(BRACKET_WIDTH_MM))
    sh.text(BR_ORIGIN[0], BR_ORIGIN[1] - hh * BR_SCALE - 19.0, "EDGE BRACKET PT-07.3, FRONT VIEW   SCALE 1:2",
            size=d.FONT_NOTE, ha="center", weight="bold")

    def bcall(target, text, ty):
        t = bp.P(*B(*target))
        return bp.leader(B(*target), text, CALL_X - t[0] - d.LEADER_SHOULDER - d.LEADER_TEXT_GAP, ty - t[1], size=d.FONT_MIN)

    bcall((BAR_CX + SCREW_CBORE_D / 2, BRACKET_SCREW_Y_MM),
          f"3 \u00d7 M5 \u00d7 {c.fmt(SCREW_LEN)} DIN 7984 SCREWS:\nCBORE \u00d8{c.fmt(SCREW_CBORE_D)} \u00d7 {c.fmt(SCREW_CBORE_DEPTH)} FROM THE BACK,\n"
          f"\u00d8{c.fmt(SCREW_CLEAR_D)} THRU, AT y = 0, \u00b1{c.fmt(BRACKET_SCREW_Y_MM)}", 208.0)
    p0 = c.spigot_hole_points(SPIGOT_CENTER)[0]
    bcall((p0[0] + c.M5_MAJOR_MM / 2, p0[1]),
          f"4 \u00d7 {sp.SPIGOT_SCREW} THRU-TAPPED (PT-02),\n\u00d8{c.fmt(sp.SPIGOT_PCD_MM)} PCD AT {', '.join(c.fmt(a) for a in sp.SPIGOT_HOLE_ANGLES_DEG)}\u00b0\n"
          f"FROM +x; CENTER x = {SPIGOT_CENTER[0]:g}", 191.0)
    bcall((SPIGOT_CENTER[0] + sp.SPIGOT_FLANGE_DIAMETER_MM / 2 * math.cos(math.radians(150)),
           SPIGOT_CENTER[1] + sp.SPIGOT_FLANGE_DIAMETER_MM / 2 * math.sin(math.radians(150))),
          f"\u00d8{c.fmt(sp.SPIGOT_FLANGE_DIAMETER_MM)} FLANGE (PHANTOM)\nON THE BACK FACE", 176.0)
    bcall((ARROW_PTS[4][0], ARROW_PTS[4][1]),
          f"ARROW ON THE BAR'S BACK FACE:\n{c.fmt(sp.ORIENTATION_MARK_WIDTH_MM)} WIDE \u00d7 {c.fmt(sp.ORIENTATION_MARK_DEPTH_MM)} DEEP, "
          f"POINTS {sp.SPIGOT_DOWEL_DIRECTION}", 162.0)
    sv = View(sh, SIDE_ORIGIN, SIDE_SCALE)

    def S(x, z):
        return (x - BR_CX, -z)

    zt_ = BRACKET_THICKNESS_MM
    sv.polyline([S(EXT_X0, 0), S(BAR_X1, 0), S(BAR_X1, zt_), S(EXT_X0, zt_)], "outline", closed=True)
    sv.polyline([S(BAR_CX - cl, 0), S(BAR_CX - cl, zt_ - SCREW_CBORE_DEPTH),
                 S(BAR_CX - cb, zt_ - SCREW_CBORE_DEPTH), S(BAR_CX - cb, zt_)], "hidden")
    sv.polyline([S(BAR_CX + cl, 0), S(BAR_CX + cl, zt_ - SCREW_CBORE_DEPTH),
                 S(BAR_CX + cb, zt_ - SCREW_CBORE_DEPTH), S(BAR_CX + cb, zt_)], "hidden")
    for p in c.spigot_hole_points(SPIGOT_CENTER):
        for dx in (-c.M5_MAJOR_MM / 2, c.M5_MAJOR_MM / 2):
            sv.line(S(p[0] + dx, 0), S(p[0] + dx, zt_), "hidden")
    for x in (SPIGOT_CENTER[0], BAR_CX):
        sv.line(S(x, -3.0), S(x, zt_ + 3.0), "center")
    sv.dim_v(BAR_X1 - BR_CX, BAR_X1 - BR_CX, 0.0, -zt_, 8.0, c.fmt(BRACKET_THICKNESS_MM))
    sh.text(SIDE_ORIGIN[0], 69.5, "SIDE VIEW (FROM BELOW)   SCALE 1:1", size=d.FONT_NOTE, ha="center", weight="bold")
    sh.text(SIDE_ORIGIN[0] - 70.0, SIDE_ORIGIN[1] + 3.0, "FRONT (AGAINST THE PLATE)", size=d.FONT_MIN)

    # ---------------- boxed check and notes ----------------
    margins = {gp: NEAREST_CUTOUT_EDGE_MM - c.hidden_threshold_mm(gp) for gp in (GAP_SMALL_MM, GAP_LARGE_MM)}
    sh.boxed_note(BOX_X, BOX_TOP, BOX_W,
                  f"HIDDEN-STANDOFF CHECK: nearest standoff axis to a cutout edge {NEAREST_CUTOUT_EDGE_MM:.1f} mm; limit at "
                  f"the {c.fmt(GAP_LARGE_MM)} mm gap = {c.fmt(GAP_LARGE_MM)} × tan {c.fmt(c.HIDE_RAY_ANGLE_DEG)}° + "
                  f"{c.fmt(c.HIDE_MARGIN_MM)} = {c.hidden_threshold_mm(GAP_LARGE_MM):.1f} mm; margin {margins[GAP_LARGE_MM]:.1f} mm. OK.",
                  color=d.BLACK, size=d.FONT_MIN, weight="normal")
    notes = [
        f"PT-07.1 front plate {c.fmt(PLATE_W)} \u00d7 {c.fmt(PLATE_H)} \u00d7 {c.fmt(FRONT_T)}; PT-07.2 back plate {c.fmt(PLATE_W)} \u00d7 "
        f"{c.fmt(PLATE_H)} \u00d7 {c.fmt(BACK_T)}, plain, REMOVABLE (open-background captures), no spigot; PT-07.3 bracket. "
        f"Outlines rounded up to whole mm; sites per the table, position \u00b1{c.fmt(c.POSITION_TOL_MM)}.",
        f"Knife edges: countersunk FROM THE BACK at {c.fmt(c.BEVEL_DEG)}\u00b0 ({c.fmt(2 * c.BEVEL_DEG)}\u00b0 included), land {c.LAND_TEXT} mm "
        f"(max {c.fmt(c.EDGE_LAND_MAX_MM)}); record the measured land. Blank sites: no feature, keep free of marks.",
        f"Fine glass-bead blast, uniform, every face the sensor sees (also the bracket extension's front face); all plates and "
        f"disks blasted in one batch, same medium and pressure. Flatness {c.fmt(c.FLATNESS_MM)} over each front face.",
        f"Standoffs PT-03, both sets delivered; four, at x = {c.fmt(PLATE_W / 2 - CORNER_STANDOFF_INSET_MM)} and {c.fmt(LEFT_STANDOFF_X_MM)}, "
        f"y = \u00b1{c.fmt(STANDOFF_Y_MM)}. G ({c.fmt(GAP_SMALL_MM)} or {c.fmt(GAP_LARGE_MM)}) is the depth step between the front faces of the front "
        f"plate and the back plate; body = G \u2212 {c.fmt(FRONT_T)}: {c.fmt(c.standoff_body_length(GAP_SMALL_MM, FRONT_T))} or "
        f"{c.fmt(c.standoff_body_length(GAP_LARGE_MM, FRONT_T))}. Front plate: M5 through-tapped; the {c.fmt(sp.STANDOFF_STUD_FRONT_LENGTH_MM)} mm front stud "
        f"ends 0.5 below the front face; nothing may protrude the front face; plate bead-blasted before assembly. Back plate: M5 through-tapped, "
        f"8 mm studs not proud of the back face.",
        f"The bracket (8 mm) lies between the plates, against the back of the front plate ({c.fmt(BRACKET_CLEARANCE_MM)} mm clear of the back plate at "
        f"G = {c.fmt(GAP_SMALL_MM)}), outer face flush with the left edge; nothing sits behind the cutouts. The robot holds T5 by the spigot on the "
        f"extension. Screws M5 \u00d7 {c.fmt(SCREW_LEN)} DIN 7984, M5 through-tapped holes: engagement {SCREW_ENGAGEMENT:g}, tip "
        f"{SCREW_TIP_BELOW_FRONT:g} mm below the front face.",
        f"Spigot PT-02 on the extension: 4 \u00d7 M5 through-tapped, \u00d8{c.fmt(sp.SPIGOT_PCD_MM)} PCD at "
        f"{', '.join(c.fmt(a) for a in sp.SPIGOT_HOLE_ANGLES_DEG)}\u00b0 from +x, centered at x = {SPIGOT_CENTER[0]:g}; arrow "
        f"{c.fmt(sp.ORIENTATION_MARK_WIDTH_MM)} \u00d7 {c.fmt(sp.ORIENTATION_MARK_DEPTH_MM)} deep, pointing {sp.SPIGOT_DOWEL_DIRECTION}, on the "
        f"bar's back face at x = {c.fmt(BAR_CX)}.",
    ]
    bottom = sh.notes_block(NOTES_X, NOTES_TOP, NOTES_W, notes, size=c.NOTE_SIZE)
    assert bottom > d.FRAME_BOTTOM, f"notes overflow the sheet (bottom {bottom:.1f})"
    sh.title_block(TitleInfo("PT-07", "T5 cutout plate, back plate and edge bracket", "1 + 1 + 1",
                             c.MATERIAL_TEXT, c.FINISH_TEXT, "1:2, 1:1, 4:1"))
    path = os.path.join(out_dir, "PT-07_T5_cutout_plate.png")
    issues = sh.save(path)
    c.report_sheet("PT-07", path, issues)
    return issues


def print_checks() -> None:
    print("PT-07", *c.check_against_part_scripts())
    print(f"PT-07 plates {PLATE_W:g} x {PLATE_H:g}; bracket bar x {BAR_X0:g}..{BAR_X1:g}, extension to {EXT_X0:g}, spigot at {SPIGOT_CENTER}")
    for f in CUTOUTS:
        print(f"PT-07 {f.site_id}: front D {f.diameter_mm:.3f}, countersink back D {csk_back_diameter(f):.3f}")
    print(f"PT-07 bracket screw: max length {SCREW_MAX_LEN:g} -> M5 x {SCREW_LEN}; engagement {SCREW_ENGAGEMENT:g}, tip "
          f"{SCREW_TIP_BELOW_FRONT:g} below the front face, head {SCREW_HEAD_RECESS:g} below the bracket's back face "
          f"(M5 x {SCREW_LENGTHS[SCREW_LENGTHS.index(SCREW_LEN) + 1]} would protrude "
          f"{SCREW_LENGTHS[SCREW_LENGTHS.index(SCREW_LEN) + 1] - BRACKET_WEB - FRONT_T:g} mm)")
    for gp in (GAP_SMALL_MM, GAP_LARGE_MM):
        print(f"PT-07 hidden check, G {gp:g}: nearest standoff axis to a cutout edge {NEAREST_CUTOUT_EDGE_MM:.2f} mm vs G x tan30 + 5 = "
              f"{c.hidden_threshold_mm(gp):.2f} mm (margin {NEAREST_CUTOUT_EDGE_MM - c.hidden_threshold_mm(gp):.2f})")
    print(f"PT-07 standoff bodies: {c.standoff_label(GAP_SMALL_MM, GAP_LARGE_MM, FRONT_T)} ({c.body_length_source()}); "
          f"bracket {BRACKET_THICKNESS_MM:g} mm leaves {BRACKET_CLEARANCE_MM:g} mm to the back plate at G = {GAP_SMALL_MM:g}")
    print(f"PT-07 bracket inner edge to the nearest cutout edge {BRACKET_TO_CUTOUT_MM:.2f} mm (nothing behind the cutouts); "
          f"left standoff body to the bracket {(LEFT_STANDOFF_X_MM - BODY_D / 2) - BAR_X1:.1f} mm")
    blank_margin = min(math.hypot(p[0] - b.x_mm, p[1] - b.y_mm) - b.diameter_mm / 2 - 0 for p in STANDOFFS + SCREW_POINTS for b in BLANKS)
    print(f"PT-07 nearest standoff or screw hole axis to a blank patch edge: {blank_margin:.1f} mm")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    print_checks()
    raise SystemExit(1 if build(here) else 0)
