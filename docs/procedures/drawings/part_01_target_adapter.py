"""Shop drawing PT-01: target adapter (square plate, ISO 9409-1-50-4-M6 flange side, spigot bore on the target side).

Frame.  Plan view: looked at from the FRONT (target side), x to the right (+Xm), y up (+Ym), origin on the
bore axis.  Depth z is measured from the front face toward the back (flange) face.

Section A-A: cutting plane x = 0 (vertical line in the plan), seen toward -X.  In the view, u = depth z
(to the right), v = y (up).  It shows the bore, the keyway at +Ym, the lead chamfer, the spigot and (as a
circle on the far bore wall) the 9 mm clearance end of the cross hole.

Section B-B: cutting plane y = 0 (horizontal line in the plan), seen toward +Y.  It is TURNED 90 degrees
counterclockwise so that it is laid out like A-A: u = depth z (to the right), v = x (up, so +Xm is up).  It
shows the cross hole in full length, the dowel hole with its pin and the spigot.

Checks that the drafting helper cannot make are computed here (``geometry_checks``) and printed by
``build`` so that a change of a constant that makes holes collide is noticed.
"""

from __future__ import annotations

import math
import os

import drafting as d
from drafting import RED, Sheet, TitleInfo, View, polar, thread_minor_diameter

# ---------------------------------------------------------------------------
# Part dimensions (mm).  Constants exported to the sister scripts are in the first block.
# ---------------------------------------------------------------------------
NUMBER = "PT-01"  # drawing number
PART_NAME = "Target adapter"  # part name in the title block
FILE_NAME = "PT-01_target_adapter.png"  # output file
QUANTITY = "1"  # off

PLATE_W = 120.0  # plate length along x (chosen, confirm with the shop)
PLATE_H = 120.0  # plate height along y (chosen)
PLATE_T = 30.0  # plate thickness (chosen)

# robot-flange side (back), exactly as on stage-1 SC1-05 and SC1-01
FLANGE_PCD = 50.0  # ISO 9409-1-50-4-M6 bolt circle diameter d1
SPIGOT_D = 31.5  # back-side centering spigot diameter
SPIGOT_H = 5.0  # spigot height
SPIGOT_CHAMFER = 0.5  # spigot chamfer 0.5 x 45
DOWEL_D = 6.0  # dowel pin and hole diameter
DOWEL_PROTRUSION = 5.0  # dowel pin protrusion beyond the back face
DOWEL_HOLE_DEPTH = 8.0  # blind dowel hole, full diameter (chosen: 1.3 d of press fit, short enough to clear the cross hole)
DOWEL_PIN_LEN = DOWEL_HOLE_DEPTH + DOWEL_PROTRUSION  # pin length, 13 (cut from 14 mm stock or ground)
DRILL_POINT_FACTOR = 0.3  # drill point depth as a fraction of the diameter (118 degree point)
BOLT_HOLE_D = 6.6  # clearance hole for M6
CBORE_D = 11.0  # counterbore diameter
CBORE_DEPTH = 6.5  # counterbore depth, from the FRONT
BOLT_ANGLES = (45.0, 135.0, 225.0, 315.0)  # bolt angles from the pin direction (+Xm)
POSITION_TOL = 0.1  # diameter position tolerance of the bolt holes
FLANGE_SCREW_ENGAGEMENT = 6.5  # thread engagement in the robot flange, as on SC1-01, SC1-02 and SC1-05
# M6 DIN 7984 screw length: grip under the counterbore plus the engagement
FLANGE_SCREW_LEN = PLATE_T - CBORE_DEPTH + FLANGE_SCREW_ENGAGEMENT
FLANGE_SCREW_HEAD_D = 10.0  # M6 DIN 7984 head diameter

# target side (front)
BORE_D = 40.0  # central bore for the spigot of PT-02
BORE_DEPTH = 25.0  # bore depth (chosen)
SPIGOT_BODY_LEN = 24.0  # length of the body of the PT-02 spigot (part_02 asserts it equals its own constant)
SPIGOT_BODY_TOL = "h6"  # tolerance of the PT-02 body
BORE_CHAMFER = 1.0  # lead chamfer 1 x 45
KEY_W = 6.0  # keyway width
KEY_DEPTH = 4.0  # keyway depth below the bore wall, along the whole bore depth
CROSS_Z = 15.0  # cross hole axis depth below the front face
CROSS_D = 8.0  # cross hole diameter on the +Xm side, for the 8 mm ball-lock pin
CROSS_FAR_D = 9.0  # clearance continuation on the far (-Xm) side
HOLD_PITCH = 80.0  # square pitch of the hold-down tapped holes
HOLD_THREAD_D = 5.0  # M5
HOLD_THREAD_PITCH = 0.8  # coarse pitch of M5
HOLD_DEPTH = 10.0  # thread depth (chosen)
HOLD_DRILL_D = 4.2  # tap drill of M5
HOLD_DRILL_EXTRA = 2.0  # drill deeper than the thread (chosen)

# tolerances and datum
TOL_SPIGOT = "g6"  # spigot tolerance
TOL_DOWEL_HOLE = "H7"  # dowel hole tolerance
TOL_PIN = "m6"  # pin tolerance
TOL_BORE = "H7"  # bore tolerance
TOL_KEY = "H7"  # keyway width tolerance
TOL_CROSS = "H7"  # cross hole tolerance
TOL_PERP_BORE = "0.02"  # bore axis perpendicular to the back face ...
PERP_LENGTH = 100.0  # ... per this axis length
DATUM_BACK = "C"  # back (flange) face
BORE_CLEAR_MARGIN = 2.0  # least acceptable wall between the cross hole and any other hole (check only)

MATERIAL = "Aluminum 6061-T6"
FINISH = "Black anodize or matte paint"
HARD_ANODIZE_NOTE = "hard anodize optional"

# derived
BORE_R = BORE_D / 2  # bore radius
KEY_BOTTOM_R = BORE_R + KEY_DEPTH  # radius of the keyway bottom
PCD_R = FLANGE_PCD / 2  # bolt circle radius
SPIGOT_R = SPIGOT_D / 2  # spigot radius
CROSS_R = CROSS_D / 2  # cross hole radius
CROSS_FAR_R = CROSS_FAR_D / 2  # far-side hole radius
CBORE_R = CBORE_D / 2  # counterbore radius
BOLT_R = BOLT_HOLE_D / 2  # clearance hole radius
HOLD_R = HOLD_PITCH / 2  # hold-down holes at (+-HOLD_R, +-HOLD_R)
BORE_FLOOR = PLATE_T - BORE_DEPTH  # material left behind the bore (before the spigot)
KEY_ACROSS = BORE_D + KEY_DEPTH  # keyway bottom measured from the opposite bore wall
HALF_W = PLATE_W / 2  # half plate length
HALF_H = PLATE_H / 2  # half plate height
HOLD_DRILL_DEPTH = HOLD_DEPTH + HOLD_DRILL_EXTRA  # drilled depth of the tapped holes
SPIGOT_BACK_Z = PLATE_T + SPIGOT_H  # depth of the spigot end


def cross_hole_wall() -> float:
    """Least distance between the cross hole and any counterbore or M6 clearance hole."""
    pts = [polar((0.0, 0.0), PCD_R, a) for a in BOLT_ANGLES]
    r_max = max(CROSS_R, CROSS_FAR_R)
    least = float("inf")
    for (_, y) in pts:
        for rad, z0, z1 in ((CBORE_R, 0.0, CBORE_DEPTH), (BOLT_R, 0.0, PLATE_T)):
            dy = max(abs(y) - rad, 0.0)  # nearest y of the hole's bounding rectangle in the (y, z) plane
            dz = max(z0 - CROSS_Z, CROSS_Z - z1, 0.0)
            least = min(least, math.hypot(dy, dz) - r_max)
    return least


def geometry_checks() -> list[str]:
    """Collision checks between holes; returns report lines and raises if one is violated."""
    lines: list[str] = []
    pts = [polar((0.0, 0.0), PCD_R, a) for a in BOLT_ANGLES]
    # 1. cross hole axis: along x at y = 0, depth CROSS_Z.  Counterbore: cylinder along z, depth 0..CBORE_DEPTH.
    #    In the (y, z) plane the cross hole is a circle (radius r_max) about (0, CROSS_Z).
    r_max = max(CROSS_R, CROSS_FAR_R)
    worst = None
    for (x, y) in pts:
        for rad, z0, z1, what in ((CBORE_R, 0.0, CBORE_DEPTH, "counterbore"), (BOLT_R, 0.0, PLATE_T, "clearance hole")):
            dy = max(abs(y) - rad, 0.0)  # nearest y of the hole's bounding rectangle
            dz = max(z0 - CROSS_Z, CROSS_Z - z1, 0.0)
            gap = math.hypot(dy, dz) - r_max
            if worst is None or gap < worst[0]:
                worst = (gap, what)
    lines.append(f"cross hole to counterbores and bolt holes: least wall {worst[0]:.1f} ({worst[1]})")
    if worst[0] < BORE_CLEAR_MARGIN:
        raise ValueError("cross hole too close to the bolt holes")
    # 2. cross hole against the dowel hole (blind from the back, at x = PCD_R on y = 0): web in z
    cross_bottom = CROSS_Z + CROSS_R  # lowest (deepest) point of the 8 mm hole
    dowel_full = PLATE_T - DOWEL_HOLE_DEPTH  # depth where the full-diameter dowel hole ends
    dowel_point = dowel_full - DRILL_POINT_FACTOR * DOWEL_D  # drill point tip
    lines.append(f"dowel hole to cross hole: web {dowel_full - cross_bottom:.1f} at full diameter, "
                 f"{dowel_point - cross_bottom:.1f} at the drill point")
    if dowel_point - cross_bottom < 1.0:
        raise ValueError("dowel hole drill point too close to the cross hole")
    # 3. cross hole against the bore floor
    lines.append(f"cross hole to bore floor: {BORE_DEPTH - (CROSS_Z + CROSS_FAR_R):.1f}")
    # 4. counterbores against the bore wall (known: they merge)
    inner = PCD_R - CBORE_R
    lines.append(f"counterbore inner edge at r = {inner:.2f} against bore radius {BORE_R:.1f}: "
                 f"{'MERGES with the bore by %.2f' % (BORE_R - inner) if inner < BORE_R else 'clear'}")
    half_chord = (math.sqrt(max(BORE_R ** 2 - ((PCD_R ** 2 + BORE_R ** 2 - CBORE_R ** 2) / (2 * PCD_R)) ** 2, 0.0))
                  if inner < BORE_R else 0.0)
    lines.append(f"  notch in the bore wall: {2 * half_chord:.1f} wide, {CBORE_DEPTH:.1f} deep, 4 places")
    # 5. M6 screw heads against the bore wall
    lines.append(f"M6 head edge at r = {PCD_R - FLANGE_SCREW_HEAD_D / 2:.1f} (bore wall at {BORE_R:.1f})")
    # 6. hold-down holes against everything
    lines.append(f"hold-down holes at r = {HOLD_R * math.sqrt(2):.1f}; counterbore outer edge at {PCD_R + CBORE_R:.1f}")
    lines.append(f"floor behind the bore {BORE_FLOOR:.1f}; keyway bottom radius {KEY_BOTTOM_R:.1f}")
    return lines


# ---------------------------------------------------------------------------
# Sheet layout (paper mm)
# ---------------------------------------------------------------------------
SCALE = 1.0  # all views 1:1
SCALE_TEXT = "1:1"
PLAN_CENTER = (78.0, 176.0)  # paper position of the bore axis in the plan
PLAN_LABEL = (15.0, 253.0)  # left end and baseline of the plan label
PLAN_CENTER_EXT = 5.0  # plan centerlines extend this far past the plate (model)
CUT_END = 66.0  # cutting-plane ends are this far from the axis (model)
CUT_A_SIGHT = (-1.0, 0.0)  # A-A is seen toward -X
CUT_B_SIGHT = (0.0, 1.0)  # B-B is seen toward +Y
DIM_OVERALL_OFFSET = -9.0  # overall width dimension below the plate (paper)
DIM_HEIGHT_OFFSET = 10.0  # overall height dimension right of the plate (paper)
DIM_HOLD_H_OFFSET = -9.0  # hold-down pitch dimension below the lower hole row (paper)
DIM_HOLD_V_OFFSET = -10.0  # hold-down pitch dimension left of the left hole column (paper)
DIM_KEY_OFFSET = 7.0  # keyway width dimension above the keyway bottom (paper)
ANGLE_DIM_R = 33.0  # radius of the 45 degree dimension arc (paper = model at 1:1)
ANGLE_TEXT_DX = 6.0  # 45 degree text offset from the arc middle (paper)
ANGLE_EXT_FROM = 29.0  # start radius of the 45 degree extension line
ANGLE_EXT_TO = 36.0  # end radius of the 45 degree extension line
XM_ARROW = ((33.0, -12.0), (53.0, -12.0))  # +Xm key arrow start and end (model, plan)
XM_LABEL_DY = -5.0  # +Xm label baseline relative to its arrow (paper)
CALLOUT_X = 156.0  # left edge of the plan callout text column (paper)
CALLOUT_ELBOW_GAP = 5.0  # elbow of a plan leader this far left of the text column (paper)
CALLOUT_Y = {"hold": 238.0, "cbore": 214.0, "pcd": 178.0, "dowel": 162.0, "spigot": 146.0,
             "bore": 128.0}  # text baseline y of each plan callout (paper)
CBORE_LEADER_ANGLE = 40.0  # leader contact angle on the counterbore
PCD_LEADER_ANGLE = 15.0  # leader contact angle on the pitch circle
DOWEL_LEADER_ANGLE = 340.0  # leader contact angle on the dowel hole
SPIGOT_LEADER_ANGLE = 300.0  # leader contact angle on the hidden spigot circle
BORE_LEADER_ANGLE = 290.0  # leader contact angle on the bore
HOLD_LEADER_ANGLE = 45.0  # leader contact angle on a hold-down hole
FRAME_DY = -7.0  # position frame offset below the counterbore callout (paper)
THREAD_ARC_END_DEG = 270.0  # thread major circle drawn as a three-quarter arc

A_ORIGIN = (236.0, 176.0)  # paper position of (u = 0, v = 0) in section A-A
B_ORIGIN = (316.0, 176.0)  # paper position of (u = 0, v = 0) in section B-B
A_LABEL = (253.0, 253.0)  # center and baseline of the A-A label
B_LABEL = (346.0, 253.0)  # center and baseline of the B-B label
A_BOTTOM_DIM = -9.0  # plate thickness dimension offset below the section (paper)
A_SPIGOT_DIM = -9.0  # spigot height dimension offset below the spigot (paper)
A_DIA_DIM = 9.0  # spigot diameter dimension offset right of the spigot end (paper)
A_BORE_DEPTH_V = -13.0  # v of the bore depth dimension line inside the bore (model)
A_BORE_DIA_U = 22.0  # u of the bore diameter dimension line inside the bore (model)
A_BORE_DIA_TEXT_POS = 0.8  # bore diameter text position along its dimension line
B_DIM_BOTTOM = -9.0  # cross hole depth dimension offset below the section (paper)
B_PIN_DIM = 10.0  # pin protrusion dimension offset right of the back face (paper)
B_PIN_POS_DIM = 16.0  # dowel position dimension offset right of the back face (paper)
B_CALLOUT_X = 374.0  # left edge of the B-B right-hand callout text column (paper)
B_CROSS_LEADER = (8.0, 14.0)  # elbow offset (paper) of the 8 mm cross hole callout above the section
B_FAR_Y = 126.0  # text y (paper) of the 9 mm clearance callout
B_PIN_Y = 196.0  # text y (paper) of the dowel pin callout
KEY_CALLOUT_Y = 246.0  # text y (paper) of the keyway callout above section A-A
KEY_CALLOUT_DX = 6.0  # elbow offset of the keyway callout (paper)
B_KEY_ARROW = ((-10.0, 200.0), (-10.0, 216.0))  # +Xm arrow left of B-B: (dx from origin, y paper) start and end
DATUM_C_V = -45.0  # v where datum C is attached to the back face in A-A

FLANGE_NOTE_POS = (245.0, 98.0)  # top-left of the boxed flange note
FLANGE_NOTE_W = 162.0  # width of the boxed flange note
NOTES_X = 12.0  # notes block left edge
NOTES_TOP = 99.0  # notes block top
NOTES_COLUMN_W = 72.0  # width of each notes column
NOTES_COLUMN_GAP = 4.0  # gap between notes columns
NOTES_COLUMNS = 3  # number of notes columns
NOTES_SIZE = 9.0  # notes text size (the minimum allowed)
TAG_SIZE = 10.0  # size of feature tags
WHITE_MASK_Z = 3.4  # z-order of white masks


def _fmt(x: float) -> str:
    """Format a number without trailing zeros."""
    return f"{x:g}"


def build(out_dir: str) -> list[str]:
    """Draw PT-01 and save it.  Returns the QA issues found."""
    for line in geometry_checks():
        print(f"  [{NUMBER}] {line}")
    sh = Sheet(NUMBER)
    sh.border()
    pr, rs, rb = PCD_R, SPIGOT_R, BORE_R
    cb_pts = [polar((0.0, 0.0), pr, a) for a in BOLT_ANGLES]
    hold_pts = [(sx * HOLD_R, sy * HOLD_R) for sx in (-1, 1) for sy in (-1, 1)]
    key_half = KEY_W / 2
    key_wall_y = math.sqrt(rb ** 2 - key_half ** 2)  # where the keyway edge meets the bore wall
    eps = 1e-6

    # =====================================================================
    # PLAN VIEW FROM THE FRONT
    # =====================================================================
    plan = View(sh, PLAN_CENTER, SCALE)
    plan.polyline([(-HALF_W, -HALF_H), (HALF_W, -HALF_H), (HALF_W, HALF_H), (-HALF_W, HALF_H)], "outline", closed=True)
    plan.center_h(-HALF_W - PLAN_CENTER_EXT, HALF_W + PLAN_CENTER_EXT, 0.0)
    plan.center_v(-HALF_H - PLAN_CENTER_EXT, HALF_H + PLAN_CENTER_EXT, 0.0)
    plan.circle((0, 0), pr, "center")

    def in_cbore(u: float, v: float) -> bool:
        return any(math.hypot(u - cx, v - cy) < CBORE_R - eps for cx, cy in cb_pts)

    def in_key(u: float, v: float) -> bool:
        return abs(u) < key_half - eps and v > 0

    # bore and lead chamfer, interrupted by the keyway and by the counterbores that merge with the bore
    plan.circle_except((0, 0), rb, [in_cbore, in_key], "outline")
    plan.circle_except((0, 0), rb + BORE_CHAMFER, [in_cbore, in_key], "outline")
    for sx in (-1, 1):  # keyway edges and bottom
        plan.line((sx * key_half, key_wall_y), (sx * key_half, KEY_BOTTOM_R), "outline")
    plan.line((-key_half, KEY_BOTTOM_R), (key_half, KEY_BOTTOM_R), "outline")
    plan.circle((0, 0), rs, "hidden")  # back-side spigot
    plan.circle((pr, 0.0), DOWEL_D / 2, "hidden")  # back-side dowel hole at +Xm
    for p in cb_pts:  # counterbored holes, visible from the front
        plan.circle(p, BOLT_R, "outline")
        plan.circle_except(p, CBORE_R, [lambda u, v: math.hypot(u, v) < rb], "outline")
        plan.center_cross(p, CBORE_R)
    r_minor = thread_minor_diameter(HOLD_THREAD_D, HOLD_THREAD_PITCH) / 2
    for p in hold_pts:  # hold-down tapped holes: minor circle thick, major circle three-quarter thin arc
        plan.circle(p, r_minor, "outline")
        plan.circle(p, HOLD_THREAD_D / 2, "thin", 0.0, THREAD_ARC_END_DEG)
        plan.center_cross(p, HOLD_THREAD_D / 2)
    # +Xm key (red)
    (kx0, ky0), (kx1, ky1) = XM_ARROW
    a0, a1 = plan.P(kx0, ky0), plan.P(kx1, ky1)
    sh.line([a0, a1], "outline", RED)
    sh.arrow(a1, (1, 0), RED)
    sh.text(a0[0], a0[1] + XM_LABEL_DY, "+Xm", size=d.FONT_NOTE, color=RED, weight="bold")
    # cutting planes
    plan.cutting_plane([(0.0, CUT_END), (0.0, -CUT_END)], sight=CUT_A_SIGHT, label="A")
    plan.cutting_plane([(-CUT_END, 0.0), (CUT_END, 0.0)], sight=CUT_B_SIGHT, label="B")

    # ---- plan dimensions ----------------------------------------------------
    plan.dim_h(-HALF_W, HALF_W, -HALF_H, -HALF_H, DIM_OVERALL_OFFSET, _fmt(PLATE_W), text_pos=0.2)
    plan.dim_v(HALF_W, HALF_W, -HALF_H, HALF_H, DIM_HEIGHT_OFFSET, _fmt(PLATE_H), text_pos=0.2)
    yh = -HOLD_R - HOLD_THREAD_D / 2  # extension lines start below the lower hole circles
    plan.dim_h(-HOLD_R, HOLD_R, yh, yh, DIM_HOLD_H_OFFSET, _fmt(HOLD_PITCH), text_pos=0.3)
    xh = -HOLD_R - HOLD_THREAD_D / 2
    plan.dim_v(xh, xh, -HOLD_R, HOLD_R, DIM_HOLD_V_OFFSET, _fmt(HOLD_PITCH), text_pos=0.3)
    plan.dim_h(-key_half, key_half, KEY_BOTTOM_R, KEY_BOTTOM_R, DIM_KEY_OFFSET,
               f"{_fmt(KEY_W)} {TOL_KEY} × {_fmt(KEY_DEPTH)} DEEP", outside="left")
    plan.dim_angle((0, 0), 0.0, BOLT_ANGLES[0], ANGLE_DIM_R, "45°", text_dx=ANGLE_TEXT_DX)
    a_mid = BOLT_ANGLES[0]
    sh.line([plan.P(*polar((0, 0), ANGLE_EXT_FROM, a_mid)), plan.P(*polar((0, 0), ANGLE_EXT_TO, a_mid))], "thin")

    # ---- plan callouts: right-hand column --------------------------------------
    elbow_dx = lambda p_paper: CALLOUT_X - CALLOUT_ELBOW_GAP - p_paper[0]  # noqa: E731  (dx to the column)
    hp = plan.P(*polar(hold_pts[3], HOLD_THREAD_D / 2, HOLD_LEADER_ANGLE))
    plan.leader_circle(hold_pts[3], HOLD_THREAD_D / 2, HOLD_LEADER_ANGLE,
                       f"4 × M5 × {_fmt(HOLD_DEPTH)} DEEP (TAP DRILL\nØ{_fmt(HOLD_DRILL_D)} × {_fmt(HOLD_DRILL_DEPTH)}), "
                       f"{_fmt(HOLD_PITCH)} SQUARE,\nOPTIONAL HOLD-DOWN",
                       elbow_dx(hp), CALLOUT_Y["hold"] - hp[1])
    cp = plan.P(*polar(cb_pts[0], CBORE_R, CBORE_LEADER_ANGLE))
    box = plan.leader_circle(cb_pts[0], CBORE_R, CBORE_LEADER_ANGLE,
                             f"4 × Ø{_fmt(BOLT_HOLE_D)} THRU AT 45°, 135°,\n225°, 315° FROM +Xm\n"
                             f"CBORE Ø{_fmt(CBORE_D)} × {_fmt(CBORE_DEPTH)} DEEP\nFROM THE FRONT",
                             elbow_dx(cp), CALLOUT_Y["cbore"] - cp[1])
    sh.fcf(box[0], box[1] + FRAME_DY, "position", f"Ø{_fmt(POSITION_TOL)}", ())
    pp = plan.P(*polar((0, 0), pr, PCD_LEADER_ANGLE))
    plan.leader_circle((0, 0), pr, PCD_LEADER_ANGLE, f"Ø{_fmt(FLANGE_PCD)} PCD (BASIC)", elbow_dx(pp), CALLOUT_Y["pcd"] - pp[1])
    dp = plan.P(*polar((pr, 0.0), DOWEL_D / 2, DOWEL_LEADER_ANGLE))
    plan.leader_circle((pr, 0.0), DOWEL_D / 2, DOWEL_LEADER_ANGLE,
                       f"DOWEL HOLE Ø{_fmt(DOWEL_D)} {TOL_DOWEL_HOLE} × {_fmt(DOWEL_HOLE_DEPTH)}\nFROM THE BACK AT +Xm (HIDDEN)",
                       elbow_dx(dp), CALLOUT_Y["dowel"] - dp[1])
    sp = plan.P(*polar((0, 0), rs, SPIGOT_LEADER_ANGLE))
    plan.leader_circle((0, 0), rs, SPIGOT_LEADER_ANGLE,
                       f"SPIGOT Ø{_fmt(SPIGOT_D)} {TOL_SPIGOT} × {_fmt(SPIGOT_H)}\n{_fmt(SPIGOT_CHAMFER)} × 45° CHAMFER,\nON BACK (HIDDEN)",
                       elbow_dx(sp), CALLOUT_Y["spigot"] - sp[1])
    bp = plan.P(*polar((0, 0), rb, BORE_LEADER_ANGLE))
    bbox = plan.leader_circle((0, 0), rb, BORE_LEADER_ANGLE,
                              f"BORE Ø{_fmt(BORE_D)} {TOL_BORE} × {_fmt(BORE_DEPTH)} DEEP,\n{_fmt(BORE_CHAMFER)} × 45° LEAD CHAMFER",
                              elbow_dx(bp), CALLOUT_Y["bore"] - bp[1])
    sh.fcf(bbox[0], bbox[1] + FRAME_DY, "perp", f"{TOL_PERP_BORE} / {_fmt(PERP_LENGTH)}", (DATUM_BACK,))

    # =====================================================================
    # SECTION A-A  (plane x = 0, seen toward -X; u = depth, v = y)
    # =====================================================================
    ea = View(sh, A_ORIGIN, SCALE)
    c_sp = SPIGOT_CHAMFER
    back = SPIGOT_BACK_Z
    outline_a = [(0.0, -HALF_H), (0.0, -rb - BORE_CHAMFER), (BORE_CHAMFER, -rb), (BORE_DEPTH, -rb),
                 (BORE_DEPTH, KEY_BOTTOM_R), (0.0, KEY_BOTTOM_R), (0.0, HALF_H), (PLATE_T, HALF_H),
                 (PLATE_T, rs), (back - c_sp, rs), (back, rs - c_sp), (back, -rs + c_sp), (back - c_sp, -rs),
                 (PLATE_T, -rs), (PLATE_T, -HALF_H)]
    ea.section_region(outline_a)
    ea.circle((CROSS_Z, 0.0), CROSS_FAR_R, "outline")  # far end of the cross hole, on the far bore wall
    ea.center_h(-3.0, BORE_DEPTH, 0.0)
    ea.center_v(-CROSS_FAR_R - 3.0, CROSS_FAR_R + 3.0, CROSS_Z)
    ea.center_h(BORE_DEPTH, back + 3.0, 0.0)
    # dimensions
    ea.dim_h(0, PLATE_T, -HALF_H, -HALF_H, A_BOTTOM_DIM, _fmt(PLATE_T))
    ea.dim_h(PLATE_T, back, -rs, -rs, A_SPIGOT_DIM, _fmt(SPIGOT_H), outside="right")
    ea.dim_v(back, back, -rs, rs, A_DIA_DIM, f"Ø{_fmt(SPIGOT_D)} {TOL_SPIGOT}")
    # bore depth inside the bore, from the (extended) front plane to the floor
    ea.line((0.0, -rb - BORE_CHAMFER), (0.0, A_BORE_DEPTH_V - 3.0), "thin")
    ea.dim_h(0, BORE_DEPTH, A_BORE_DEPTH_V, A_BORE_DEPTH_V, 0.0, _fmt(BORE_DEPTH), base_v=A_BORE_DEPTH_V,
             ext0=False, ext1=False)
    ea.dim_v(A_BORE_DIA_U, A_BORE_DIA_U, -rb, rb, 0.0, f"Ø{_fmt(BORE_D)} {TOL_BORE}", base_u=A_BORE_DIA_U,
             text_pos=A_BORE_DIA_TEXT_POS, ext0=False, ext1=False)
    kc = ea.P(BORE_DEPTH / 2, KEY_BOTTOM_R)
    ea.leader((BORE_DEPTH / 2, KEY_BOTTOM_R), "KEYWAY BOTTOM (+Ym)", KEY_CALLOUT_DX, KEY_CALLOUT_Y - kc[1])
    # datum C
    sh.datum_feature(ea.P(PLATE_T, DATUM_C_V), (1.0, 0.0), DATUM_BACK)

    # =====================================================================
    # SECTION B-B  (plane y = 0, seen toward +Y, turned: u = depth, v = x)
    # =====================================================================
    eb = View(sh, B_ORIGIN, SCALE)
    cz0, cz1 = CROSS_Z - CROSS_R, CROSS_Z + CROSS_R  # 8 mm hole depth range
    fz0, fz1 = CROSS_Z - CROSS_FAR_R, CROSS_Z + CROSS_FAR_R  # 9 mm hole depth range
    dz0 = PLATE_T - DOWEL_HOLE_DEPTH  # depth of the dowel hole bottom
    dl, dh = pr - DOWEL_D / 2, pr + DOWEL_D / 2  # dowel hole v range
    front_up = [(0.0, HALF_W), (cz0, HALF_W), (cz0, rb), (BORE_CHAMFER, rb), (0.0, rb + BORE_CHAMFER)]
    front_dn = [(0.0, -HALF_W), (fz0, -HALF_W), (fz0, -rb), (BORE_CHAMFER, -rb), (0.0, -rb - BORE_CHAMFER)]
    rear = [(cz1, HALF_W), (PLATE_T, HALF_W), (PLATE_T, dh), (dz0, dh), (dz0, dl), (PLATE_T, dl),
            (PLATE_T, rs), (back - c_sp, rs), (back, rs - c_sp), (back, -rs + c_sp), (back - c_sp, -rs),
            (PLATE_T, -rs), (PLATE_T, -HALF_W), (fz1, -HALF_W), (fz1, -rb), (BORE_DEPTH, -rb),
            (BORE_DEPTH, rb), (cz1, rb)]
    for region in (front_up, front_dn, rear):
        eb.section_region(region)
    eb.section_region([(dz0, dl), (back, dl), (back, dh), (dz0, dh)], other=True)  # dowel pin
    for sx in (-1, 1):  # keyway edges seen on the far bore wall (+Ym half is kept)
        eb.line((0.0, sx * key_half), (BORE_DEPTH, sx * key_half), "outline")
    eb.center_h(-3.0, back + 3.0, 0.0)  # note: horizontal in the view = depth direction, so this is the axis
    eb.center_v(-HALF_W - 3.0, HALF_W + 3.0, CROSS_Z)  # cross hole axis (runs along v)
    eb.center_h(PLATE_T - DOWEL_HOLE_DEPTH - 3.0, back + 3.0, pr)  # dowel axis
    # dimensions
    eb.dim_h(0, CROSS_Z, -HALF_W, -HALF_W, B_DIM_BOTTOM, _fmt(CROSS_Z))
    eb.dim_h(PLATE_T, back, dh, dh, B_PIN_DIM, _fmt(DOWEL_PROTRUSION), outside="right")
    eb.dim_v(back, back, 0.0, pr, B_PIN_POS_DIM, _fmt(pr))
    # callouts: the 8 mm hole above the section, the others in the column right of the section
    eb.leader((CROSS_Z, HALF_W - 10.0), f"CROSS HOLE Ø{_fmt(CROSS_D)} {TOL_CROSS} FROM THE\n+Xm SIDE TO THE BORE", *B_CROSS_LEADER)
    fp = eb.P(CROSS_Z, -HALF_W + 10.0)
    eb.leader((CROSS_Z, -HALF_W + 10.0), f"Ø{_fmt(CROSS_FAR_D)} CLEARANCE\nON THE -Xm SIDE", B_CALLOUT_X - 5.0 - fp[0],
              B_FAR_Y - fp[1])
    pp2 = eb.P(back - 2.0, pr)
    eb.leader((back - 2.0, pr), f"DOWEL PIN\nØ{_fmt(DOWEL_D)} {TOL_PIN} × {_fmt(DOWEL_PIN_LEN)}\nIN HOLE\nØ{_fmt(DOWEL_D)} {TOL_DOWEL_HOLE} × {_fmt(DOWEL_HOLE_DEPTH)}",
              B_CALLOUT_X - 5.0 - pp2[0], B_PIN_Y - pp2[1])
    # +Xm key (red), pointing up in this turned view
    (kdx0, ky0b), (kdx1, ky1b) = B_KEY_ARROW
    sh.line([(B_ORIGIN[0] + kdx0, ky0b), (B_ORIGIN[0] + kdx1, ky1b)], "outline", RED)
    sh.arrow((B_ORIGIN[0] + kdx1, ky1b), (0, 1), RED)
    sh.text(B_ORIGIN[0] + kdx0 - 2.0, ky1b + 2.0, "+Xm", size=d.FONT_NOTE, color=RED, weight="bold", ha="right")

    # ---- boxed flange note, labels, notes, title block ----------------------------
    sh.boxed_note(FLANGE_NOTE_POS[0], FLANGE_NOTE_POS[1], FLANGE_NOTE_W, d.TEXT_FLANGE_NOTE)
    sh.text(PLAN_LABEL[0], PLAN_LABEL[1], f"PLAN VIEW FROM FRONT (TARGET SIDE)   SCALE {SCALE_TEXT}", size=d.FONT_LABEL,
            weight="bold")
    sh.text(A_LABEL[0], A_LABEL[1], f"SECTION A-A   SCALE {SCALE_TEXT}", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(B_LABEL[0], B_LABEL[1], f"SECTION B-B (TURNED)   SCALE {SCALE_TEXT}", size=d.FONT_LABEL, ha="center",
            weight="bold")
    sh.notes_columns(NOTES_X, NOTES_TOP, NOTES_COLUMN_W, NOTES_COLUMN_GAP, _notes(), NOTES_COLUMNS, size=NOTES_SIZE)
    sh.title_block(TitleInfo(NUMBER, PART_NAME, QUANTITY, MATERIAL, FINISH, SCALE_TEXT))
    return sh.save(os.path.join(out_dir, FILE_NAME))


def _notes() -> list[str]:
    """Numbered notes of the sheet, including the drafter's choices."""
    inner = PCD_R - CBORE_R
    cross_bottom = CROSS_Z + CROSS_R
    dowel_point = PLATE_T - DOWEL_HOLE_DEPTH - DRILL_POINT_FACTOR * DOWEL_D
    return [
        "Machine the back face, spigot, front face, bore and keyway in one setup.",
        f"Datum C = back face. Bore axis perpendicular to C within {TOL_PERP_BORE} per {_fmt(PERP_LENGTH)} mm of axis "
        f"length. Keyway bottom is {_fmt(KEY_ACROSS)} across the bore. The PT-02 spigot (Ø{_fmt(BORE_D)} "
        f"{SPIGOT_BODY_TOL} × {_fmt(SPIGOT_BODY_LEN)}) bottoms {_fmt(BORE_DEPTH - SPIGOT_BODY_LEN)} short of the bore "
        f"floor; its flange face seats on the front face.",
        f"Keyway depth {_fmt(KEY_DEPTH)} equals the PT-02 dowel protrusion, so the dowel tip only just clears the "
        f"keyway bottom. Chosen, confirm with the shop (suggest {_fmt(KEY_DEPTH + 0.5)}).",
        f"Cross hole Ø{_fmt(CROSS_D)} {TOL_CROSS} at {_fmt(CROSS_Z)} below the front face, from the +Xm side face through "
        f"the bore wall, square to the bore axis, on the plane y = 0; Ø{_fmt(CROSS_FAR_D)} clearance on the -Xm side. For "
        f"an {_fmt(CROSS_D)} mm ball-lock pin (chosen, confirm). Wall to the bolt holes {cross_hole_wall():.1f}.",
        f"Flange screws 4 × M6 × {_fmt(FLANGE_SCREW_LEN)} DIN 7984 (grip {_fmt(PLATE_T - CBORE_DEPTH)} + "
        f"{_fmt(FLANGE_SCREW_ENGAGEMENT)} engagement in the flange), fitted from the front. Position tolerance "
        f"Ø{_fmt(POSITION_TOL)}; Ø{_fmt(FLANGE_PCD)} and 45° are basic.",
        f"The Ø{_fmt(CBORE_D)} counterbores reach r = {inner:.1f}, {BORE_R - inner:.1f} inside the Ø{_fmt(BORE_D)} bore: "
        f"four notches {_fmt(CBORE_DEPTH)} deep in the bore wall (drawn as is). The M6 heads (Ø{_fmt(FLANGE_SCREW_HEAD_D)}) "
        f"just touch r = {BORE_R:.0f}. Chosen, confirm with the shop.",
        f"Dowel pin Ø{_fmt(DOWEL_D)} {TOL_PIN} × {_fmt(DOWEL_PIN_LEN)} pressed into the blind Ø{_fmt(DOWEL_D)} "
        f"{TOL_DOWEL_HOLE} hole ({_fmt(DOWEL_HOLE_DEPTH)} deep), {_fmt(DOWEL_PROTRUSION)} proud toward the flange. The drill "
        f"point stops {dowel_point - cross_bottom:.1f} from the cross hole: do not drill deeper.",
        f"Hold-down: 4 × M5 × {_fmt(HOLD_DEPTH)} on a {_fmt(HOLD_PITCH)} square about the bore axis (optional).",
        "A-A: plane x = 0 seen toward -X; the circle on the far bore wall is the Ø9 end of the cross hole. B-B: plane "
        "y = 0 seen toward +Y, turned 90° so that +Xm points up; keyway edges behind the plane are shown.",
        f"Mark a witness line across the adapter and the robot flange after tightening. {HARD_ANODIZE_NOTE.capitalize()} "
        f"(then mask the bore, keyway, spigot, dowel hole and cross holes, or allow for the coating build-up).",
        f"Chosen, confirm with the shop: plate {_fmt(PLATE_W)} × {_fmt(PLATE_H)} × {_fmt(PLATE_T)}, bore depth "
        f"{_fmt(BORE_DEPTH)}, dowel Ø{_fmt(DOWEL_D)} × {_fmt(DOWEL_PIN_LEN)}, cross hole depth {_fmt(CROSS_Z)}, M5 "
        f"hold-down on {_fmt(HOLD_PITCH)}, screw lengths.",
    ]


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    issues = build(here)
    print(f"{NUMBER}: {len(issues)} layout problem(s)")
    for i in issues:
        print("   ", i)
