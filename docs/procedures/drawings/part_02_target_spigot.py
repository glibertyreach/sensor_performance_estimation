"""Shop drawing PT-02: target spigot (turned stainless part: Ø40 h6 body, Ø80 mounting flange).

Frame.  Part axis z.  End view: looked at from the TIP (tip toward the viewer), x to the right, y up (+Ym,
the dowel direction).  Side view (section A-A, aligned): the part axis is horizontal, the tip at the left, the
flange at the right; u = distance from the tip, v = y.  The cutting plane is x = 0 from the top down to the
axis (dowel and cross hole) and then bends toward 225 degrees (a counterbored flange hole), which is rotated
into the plane of the section (aligned section).  The view is seen toward -X, the retained half is x < 0.

Detail M: the mating pattern on the back plate of each target, seen from the back of the plate (the same side
as the end view), at 1:1.

Constants a sister script needs (the plate drawings that carry the mating pattern or the standoffs) are in
the first block: MOUNT_*, DOWEL_DIRECTION, BACK_PLATE_T and ``mount_hole_positions``.
"""

from __future__ import annotations

import math
import os

import drafting as d
import part_01_target_adapter as adapter
from drafting import RED, Sheet, TitleInfo, View, polar, thread_minor_diameter

# ---------------------------------------------------------------------------
# Mating pattern on the target back plate (exported)
# ---------------------------------------------------------------------------
MOUNT_PCD = 60.0  # pitch circle diameter of the four mounting screws
MOUNT_THREAD_D = 5.0  # M5
MOUNT_THREAD_PITCH = 0.8  # coarse pitch of M5
MOUNT_ANGLES_FROM_DOWEL = (45.0, 135.0, 225.0, 315.0)  # hole angles measured from the dowel direction (+Ym)
DOWEL_DIRECTION = "+Ym"  # direction of the radial dowel in the target frame
BACK_PLATE_T = 8.0  # thickness of the target back plate (aluminum), given
MOUNT_ENGAGEMENT = 6.0  # screw thread engagement in the back plate, given
MOUNT_TAP_DRILL_D = 4.2  # tap drill of M5
MOUNT_TAP_POSITION_TOL = 0.1  # diameter position tolerance of the tapped pattern (chosen)


def mount_hole_positions() -> list[tuple[float, float]]:
    """Centers (x, y) of the four mounting holes in the target frame (x right, y up, +Ym = dowel direction),
    seen from the back of the plate.  Angles are measured from +Ym."""
    r = MOUNT_PCD / 2
    return [(r * math.sin(math.radians(a)), r * math.cos(math.radians(a))) for a in MOUNT_ANGLES_FROM_DOWEL]


# ---------------------------------------------------------------------------
# Part dimensions (mm)
# ---------------------------------------------------------------------------
NUMBER = "PT-02"  # drawing number
PART_NAME = "Target spigot"  # part name in the title block
FILE_NAME = "PT-02_target_spigot.png"  # output file
QUANTITY = "4"  # off (one per target)

BODY_D = adapter.BORE_D  # body diameter, fits the bore of PT-01
TOL_BODY = adapter.SPIGOT_BODY_TOL  # h6
BODY_LEN = adapter.SPIGOT_BODY_LEN  # body length from the flange face to the tip (PT-01 constant)
TIP_CHAMFER = 1.0  # tip chamfer 1 x 45
BAR_D = 82.0  # stock bar diameter (chosen: the Ø80 flange cannot come from a smaller bar)
CROSS_D = 8.2  # cross hole, clearance for the 8 mm ball-lock pin
CROSS_Z = adapter.CROSS_Z  # distance of the cross hole axis from the flange face, equals the PT-01 cross hole depth
DOWEL_D = adapter.DOWEL_D  # radial dowel diameter
DOWEL_HOLE_DEPTH = 10.0  # radial dowel hole depth below the body surface (chosen)
DOWEL_PROTRUSION = adapter.DOWEL_TIP_PROTRUSION  # dowel protrusion beyond the body surface (4, as given)
DOWEL_PIN_LEN = DOWEL_HOLE_DEPTH + DOWEL_PROTRUSION  # pin length, 14 (ISO 2338 6 x 14)
DOWEL_Z = 6.0  # distance of the dowel axis from the flange face (chosen: near the flange)
FLANGE_D = 80.0  # mounting flange diameter
MOUNT_CLEAR_D = 5.5  # clearance hole for M5
MOUNT_CBORE_D = 9.5  # counterbore diameter
MOUNT_HEAD_H = 3.0  # head height of an M5 DIN 7984 low-head screw
MOUNT_HEAD_D = 8.5  # head diameter of an M5 DIN 7984 low-head screw
MOUNT_CBORE_RECESS = 1.0  # head sits this far below the flange face (chosen)
MOUNT_CBORE_DEPTH = MOUNT_HEAD_H + MOUNT_CBORE_RECESS  # counterbore depth
MIN_WEB = 4.0  # least flange material under the counterbore (chosen)
FLANGE_T = MIN_WEB + MOUNT_CBORE_DEPTH  # flange thickness, sized from the web and the counterbore
# M5 DIN 7984 screw length: grip under the head (flange thickness minus counterbore) plus the engagement
MOUNT_SCREW_LEN_CALC = FLANGE_T - MOUNT_CBORE_DEPTH + MOUNT_ENGAGEMENT
SCREW_LENGTHS = (8, 10, 12, 16, 20)  # DIN 7984 M5 stock lengths
MOUNT_SCREW_LEN = min(l for l in SCREW_LENGTHS if l >= MOUNT_SCREW_LEN_CALC - 1e-9)  # next stock length
MOUNT_TIP_SHORT = BACK_PLATE_T - (MOUNT_SCREW_LEN - (FLANGE_T - MOUNT_CBORE_DEPTH))  # screw tip to the plate far face
TOL_PERP_FLANGE = "0.02"  # flange face perpendicular to the body axis ...
PERP_DIA = FLANGE_D  # ... over this diameter
TOL_PAR_FLANGE = "0.02"  # body-side flange face parallel to the target-side face (chosen)
TOL_PIN = adapter.TOL_PIN  # dowel pin tolerance
TOL_DOWEL_HOLE = adapter.TOL_DOWEL_HOLE  # dowel hole tolerance
DATUM_AXIS = "A"  # body axis
DATUM_FACE = "B"  # target-side flange face

MATERIAL = f"303 or 17-4 PH stainless, Ø{BAR_D:g} bar"
FINISH = "Passivated, matte; body Ø40 h6 uncoated"

# derived
BODY_R = BODY_D / 2  # body radius
FLANGE_R = FLANGE_D / 2  # flange radius
MOUNT_R = MOUNT_PCD / 2  # mounting circle radius
TOTAL_LEN = BODY_LEN + FLANGE_T  # overall length
BORE_GAP = adapter.BORE_DEPTH - BODY_LEN  # tip to bore floor in PT-01
KEY_CLEARANCE = adapter.KEY_DEPTH - DOWEL_PROTRUSION  # PT-01 keyway bottom to dowel tip, radial
CROSS_U = BODY_LEN - CROSS_Z  # u of the cross hole axis measured from the tip
DOWEL_U = BODY_LEN - DOWEL_Z  # u of the dowel axis measured from the tip
STACK_ERR = 1e-9  # numerical tolerance of the consistency checks


def consistency_checks() -> list[str]:
    """Checks between this part and PT-01 that the drafting helper cannot make; raises on a violation."""
    lines = []
    if BORE_GAP < 0.5:
        raise ValueError("spigot body does not stop short of the bore floor")
    lines.append(f"body bottoms {BORE_GAP:.1f} short of the PT-01 bore floor (bore {adapter.BORE_DEPTH:g}, body {BODY_LEN:g})")
    if abs(CROSS_Z - adapter.CROSS_Z) > STACK_ERR:
        raise ValueError("cross hole depth differs from PT-01")
    lines.append(f"cross hole at {CROSS_Z:g} from the flange face = PT-01 depth {adapter.CROSS_Z:g} below the front face")
    if CROSS_D <= adapter.CROSS_D or CROSS_D >= adapter.CROSS_FAR_D:
        raise ValueError("cross hole should be between the 8 H7 and the 9 mm holes of PT-01")
    lines.append(f"dowel tip to the PT-01 keyway bottom: radial clearance {KEY_CLEARANCE:.1f}")
    if KEY_CLEARANCE < 0.25:
        raise ValueError("dowel tip too close to the PT-01 keyway bottom")
    gap = (CROSS_U - CROSS_D / 2) - 0.0  # cross hole to tip
    lines.append(f"cross hole edge to the tip {gap:.1f}; dowel hole edge to the cross hole edge (axial) "
                 f"{(DOWEL_U - DOWEL_D / 2) - (CROSS_U + CROSS_D / 2):.1f}")
    if (DOWEL_U - DOWEL_D / 2) - (CROSS_U + CROSS_D / 2) < 1.5:
        raise ValueError("dowel hole too close to the cross hole")
    lines.append(f"counterbore outer edge at r = {MOUNT_R + MOUNT_CBORE_D / 2:.2f}, flange radius {FLANGE_R:.0f}, "
                 f"body radius {BODY_R:.0f}")
    if MOUNT_R - MOUNT_CBORE_D / 2 <= BODY_R + 1.0:
        raise ValueError("counterbore too close to the body")
    lines.append(f"M5 x {MOUNT_SCREW_LEN} screw (calculated {MOUNT_SCREW_LEN_CALC:g}): engagement "
                 f"{MOUNT_SCREW_LEN - (FLANGE_T - MOUNT_CBORE_DEPTH):g}, tip {MOUNT_TIP_SHORT:g} short of the far face of the "
                 f"{BACK_PLATE_T:g} plate")
    if MOUNT_TIP_SHORT < 0:
        raise ValueError("mounting screw too long for the back plate")
    lines.extend(_check_shared_pattern())
    return lines


def _check_shared_pattern() -> list[str]:
    """Compare the mounting pattern with ``spigot_pattern.py``, which the plate drawings import; raise on a mismatch."""
    try:
        import spigot_pattern as sp
    except ImportError:  # the plate drawings are not present: nothing to compare
        return ["spigot_pattern.py not found: mounting pattern not compared"]
    pairs = [("PCD", MOUNT_PCD, sp.SPIGOT_PCD_MM), ("thread", MOUNT_THREAD_D, sp.SPIGOT_SCREW_DIAMETER_MM),
             ("flange diameter", FLANGE_D, sp.SPIGOT_FLANGE_DIAMETER_MM)]
    for name, mine, theirs in pairs:
        if abs(mine - theirs) > STACK_ERR:
            raise ValueError(f"{name} differs from spigot_pattern.py: {mine} against {theirs}")
    if sorted(MOUNT_ANGLES_FROM_DOWEL) != sorted(sp.SPIGOT_HOLE_ANGLES_DEG):
        raise ValueError("hole angles differ from spigot_pattern.py")
    if DOWEL_DIRECTION.lower().rstrip('m') != sp.SPIGOT_DOWEL_DIRECTION.lower():  # '+Ym' against '+y'
        raise ValueError("dowel direction differs from spigot_pattern.py")
    return ["mounting pattern agrees with spigot_pattern.py"]


# ---------------------------------------------------------------------------
# Sheet layout (paper mm)
# ---------------------------------------------------------------------------
SCALE = 1.0  # all views 1:1
SCALE_TEXT = "1:1"
END_CENTER = (62.0, 180.0)  # paper position of the axis in the end view
SIDE_ORIGIN = (200.0, 180.0)  # paper position of (u = 0 at the tip, v = 0 on the axis) in section A-A
DETAIL_CENTER = (360.0, 182.0)  # paper position of the pattern center in detail M
END_LABEL = (62.0, 253.0)  # center and baseline of the end view label
SIDE_LABEL = (230.0, 253.0)  # center and baseline of the section label
DETAIL_LABEL = (358.0, 253.0)  # center and baseline of the detail label
DETAIL_LABEL2_DY = -5.0  # second label line offset
END_CENTER_EXT = 5.0  # end view centerlines extend this far past the flange (model)
CUT_TOP = 48.0  # top end of the cutting-plane line, above the axis (model)
CUT_END_R = 44.0  # radius of the lower end of the cutting-plane line (model)
CUT_LEG_ANGLE = 225.0  # angle of the bent leg of the cutting plane in the end view (degrees)
CUT_A_SIGHT = (-1.0, 0.0)  # seen toward -X
ARC_DIM_R = 44.0  # radius of the 45 degree dimension arc (paper = model), outside the flange
ARC_EXT_FROM = 41.5  # start radius of the 45 degree extension line
ARC_EXT_TO = 47.0  # end radius of the 45 degree extension line
ARC_TEXT = (4.0, 1.0)  # offset of the 45 degree text from the arc middle (paper)
END_CALLOUT_X = 116.0  # left edge of the end view callout text (paper)
END_CALLOUT_GAP = 5.0  # elbow of an end view leader this far left of the text (paper)
END_CALLOUT_Y = {"body": 188.0, "chamfer": 204.0, "pcd": 168.0, "holes": 138.0}  # callout text y (paper)
END_BODY_ANGLE = 340.0  # leader contact angles in the end view (degrees)
END_CHAMFER_ANGLE = 20.0
END_PCD_ANGLE = 355.0
END_HOLE_ANGLE = 330.0
FRAME_DY = -7.0  # position frame offset below a hole callout (paper)
OVERALL_ROW = 17.0  # overall length dimension offset above the flange (paper)
SEG_ROW = 8.0  # body and flange length dimension offset above the flange (paper)
DOWEL_Z_ROW = 5.0  # dowel position dimension offset above the pin (paper)
CROSS_DIM_OFFSET = -8.0  # cross hole position dimension offset below the body (paper)
BODY_DIM_OFFSET = -10.0  # body diameter dimension offset left of the tip (paper)
FLANGE_DIM_U = -22.0  # u of the flange diameter dimension line (left of the tip, model)
DATUM_A_U = 4.0  # u where datum A is attached to the body surface
DATUM_B_V = 20.0  # v where datum B is attached to the target-side face
FRAME_B_POS = (8.0, -52.0)  # perpendicularity frame position (u, v model) below the flange
FRAME_B_TARGET_V = -38.0  # v where the frame leader meets the target-side face
SIDE_CALLOUT_X = 250.0  # left edge of the side view callout text column (paper)
SIDE_CALLOUT_Y = {"dowel": 226.0, "cross": 192.0, "holes": 152.0}  # side callout text y (paper)
DETAIL_TEXT_END_X = 316.0  # right edge of the detail callout text (paper)
DETAIL_CALLOUT_Y = {"tap": 125.0, "pcd": 106.0}  # detail callout text y (paper)
DETAIL_TAP_ANGLE = 250.0  # leader contact angle on the lower-left tapped hole (degrees)
DETAIL_PCD_ANGLE = 270.0  # leader contact angle on the pitch circle (degrees)
KEY_ARROW_FROM = 1.0  # red +Ym arrow starts this far beyond the centerline end (model)
KEY_ARROW_LEN = 10.0  # red +Ym arrow length (model)
KEY_TEXT_DX = 2.0  # +Ym label offset (paper)
KEY_TEXT_DY = -4.0
THREAD_ARC_END_DEG = 270.0  # thread major circle drawn as a three-quarter arc

NOTES_X = 12.0  # notes block left edge
NOTES_TOP = 99.0  # notes block top
NOTES_COLUMN_W = 72.0  # width of each notes column
NOTES_COLUMN_GAP = 4.0  # gap between notes columns
NOTES_COLUMNS = 3  # number of notes columns
NOTES_SIZE = 9.0  # notes text size (the minimum allowed)
ROLL_NOTE_POS = (245.0, 84.0)  # top-left of the boxed roll and pull-out note
ROLL_NOTE_W = 162.0  # width of the boxed roll and pull-out note
ROLL_NOTE_TEXT = ("The spigot's dowel and the adapter's keyway fix the target's roll; "
                  "the ball-lock pin fixes its pull-out.")


def _fmt(x: float) -> str:
    """Format a number without trailing zeros."""
    return f"{x:g}"


def build(out_dir: str) -> list[str]:
    """Draw PT-02 and save it.  Returns the QA issues found."""
    for line in consistency_checks():
        print(f"  [{NUMBER}] {line}")
    sh = Sheet(NUMBER)
    sh.border()
    rb, rf, rm = BODY_R, FLANGE_R, MOUNT_R
    hole_pts = [polar((0.0, 0.0), rm, 90.0 - a) for a in MOUNT_ANGLES_FROM_DOWEL]  # angles from +Ym
    dh = DOWEL_D / 2

    # =====================================================================
    # END VIEW FROM THE TIP
    # =====================================================================
    ev = View(sh, END_CENTER, SCALE)
    ev.circle((0, 0), rf, "outline")  # flange
    ev.circle((0, 0), rb, "outline")  # body
    ev.circle((0, 0), rb - TIP_CHAMFER, "outline")  # tip face edge
    ev.circle((0, 0), rm, "center")
    ev.center_h(-rf - END_CENTER_EXT, rf + END_CENTER_EXT, 0.0)
    ev.center_v(-rf - END_CENTER_EXT, rf + END_CENTER_EXT, 0.0)
    for p in hole_pts:  # counterbored mounting holes, visible from the body side
        ev.circle(p, MOUNT_CLEAR_D / 2, "outline")
        ev.circle(p, MOUNT_CBORE_D / 2, "outline")
        ev.center_cross(p, MOUNT_CBORE_D / 2)
    wall = math.sqrt(rb ** 2 - dh ** 2)  # where the dowel sides leave the body circle
    for sx in (-1, 1):  # dowel pin seen end-on beside the body, and its hole (hidden)
        ev.line((sx * dh, wall), (sx * dh, rb + DOWEL_PROTRUSION), "outline")
        ev.line((sx * dh, wall), (sx * dh, rb - DOWEL_HOLE_DEPTH), "hidden")
    ev.line((-dh, rb + DOWEL_PROTRUSION), (dh, rb + DOWEL_PROTRUSION), "outline")
    ev.line((-dh, rb - DOWEL_HOLE_DEPTH), (dh, rb - DOWEL_HOLE_DEPTH), "hidden")
    cross_x = math.sqrt(rb ** 2 - (CROSS_D / 2) ** 2)
    for sy in (-1, 1):  # cross hole along x (hidden)
        ev.line((-cross_x, sy * CROSS_D / 2), (cross_x, sy * CROSS_D / 2), "hidden")
    # cutting plane A-A (aligned): the arrow of the bent leg is the left-pointing arrow turned with the leg
    leg_end = polar((0.0, 0.0), CUT_END_R, CUT_LEG_ANGLE)
    turn = math.radians(CUT_LEG_ANGLE - 270.0)  # the leg is the downward leg turned by this angle
    sight_end = (CUT_A_SIGHT[0] * math.cos(turn) - CUT_A_SIGHT[1] * math.sin(turn),
                 CUT_A_SIGHT[0] * math.sin(turn) + CUT_A_SIGHT[1] * math.cos(turn))
    ev.cutting_plane([(0.0, CUT_TOP), (0.0, 0.0), leg_end], sight=CUT_A_SIGHT, label="A", sight_end=sight_end)
    # 45 degree dimension between the dowel direction and the first hole
    a_hole = 90.0 - MOUNT_ANGLES_FROM_DOWEL[0]
    ev.dim_angle((0, 0), a_hole, 90.0, ARC_DIM_R, "45°", text_dx=ARC_TEXT[0], text_dy=ARC_TEXT[1])
    sh.line([ev.P(*polar((0, 0), ARC_EXT_FROM, a_hole)), ev.P(*polar((0, 0), ARC_EXT_TO, a_hole))], "thin")
    # callouts right of the view
    def callout_end(target: tuple[float, float], text: str, y_paper: float) -> tuple[float, float, float, float]:
        tp = ev.P(*target)
        return ev.leader(target, text, END_CALLOUT_X - END_CALLOUT_GAP - tp[0], y_paper - tp[1])
    callout_end(polar((0, 0), rb - TIP_CHAMFER / 2, END_CHAMFER_ANGLE), f"TIP CHAMFER {_fmt(TIP_CHAMFER)} × 45°",
                END_CALLOUT_Y["chamfer"])
    callout_end(polar((0, 0), rb, END_BODY_ANGLE), f"BODY Ø{_fmt(BODY_D)} {TOL_BODY}, GROUND", END_CALLOUT_Y["body"])
    callout_end(polar((0, 0), rm, END_PCD_ANGLE), f"Ø{_fmt(MOUNT_PCD)} PCD (BASIC)", END_CALLOUT_Y["pcd"])
    box = callout_end(polar(hole_pts[3], MOUNT_CBORE_D / 2, END_HOLE_ANGLE),
                      f"4 × Ø{_fmt(MOUNT_CLEAR_D)} THRU,\nCBORE Ø{_fmt(MOUNT_CBORE_D)} × {_fmt(MOUNT_CBORE_DEPTH)} DEEP\n"
                      f"FROM THE BODY SIDE,\nAT 45°, 135°, 225°,\n315° FROM +Ym", END_CALLOUT_Y["holes"])
    sh.fcf(box[0], box[1] + FRAME_DY, "position", f"Ø{_fmt(MOUNT_TAP_POSITION_TOL)}", ())

    # =====================================================================
    # SECTION A-A  (aligned, seen toward -X; u from the tip, v = y)
    # =====================================================================
    es = View(sh, SIDE_ORIGIN, SCALE)
    u_f = BODY_LEN  # flange face (body side)
    u_b = TOTAL_LEN  # flange back face (target side)
    rcb = MOUNT_CBORE_D / 2
    rcl = MOUNT_CLEAR_D / 2
    vh = rm  # the rotated hole sits at v = -rm
    outline = [(0.0, rb - TIP_CHAMFER), (0.0, -rb + TIP_CHAMFER), (TIP_CHAMFER, -rb), (u_f, -rb),
               (u_f, -vh + rcb), (u_f + MOUNT_CBORE_DEPTH, -vh + rcb), (u_f + MOUNT_CBORE_DEPTH, -vh + rcl),
               (u_b, -vh + rcl), (u_b, -vh - rcl), (u_f + MOUNT_CBORE_DEPTH, -vh - rcl),
               (u_f + MOUNT_CBORE_DEPTH, -vh - rcb), (u_f, -vh - rcb), (u_f, -rf), (u_b, -rf), (u_b, rf), (u_f, rf),
               (u_f, rb), (DOWEL_U + dh, rb), (DOWEL_U + dh, rb - DOWEL_HOLE_DEPTH),
               (DOWEL_U - dh, rb - DOWEL_HOLE_DEPTH), (DOWEL_U - dh, rb), (TIP_CHAMFER, rb)]
    es.section_region(outline)
    # cross hole: perpendicular to the section plane, so it is cut across as a circle
    c0 = es.P(CROSS_U, 0.0)
    ccirc = [polar(c0, CROSS_D / 2 * SCALE, a) for a in range(0, 361, 5)]
    sh.polygon_fill(ccirc, d.WHITE, zorder=d.Z_MASK)
    es.circle((CROSS_U, 0.0), CROSS_D / 2, "outline")
    es.section_region([(DOWEL_U - dh, rb - DOWEL_HOLE_DEPTH), (DOWEL_U + dh, rb - DOWEL_HOLE_DEPTH),
                       (DOWEL_U + dh, rb + DOWEL_PROTRUSION), (DOWEL_U - dh, rb + DOWEL_PROTRUSION)], other=True)
    es.center_h(-3.0, u_b + 3.0, 0.0)
    es.center_v(-CROSS_D / 2 - 3.0, CROSS_D / 2 + 3.0, CROSS_U)
    es.center_v(rb - DOWEL_HOLE_DEPTH - 3.0, rb + DOWEL_PROTRUSION + 3.0, DOWEL_U)
    es.center_h(u_f - 2.0, u_b + 3.0, -vh)
    # dimensions
    es.dim_h(u_f, u_b, rf, rf, SEG_ROW, _fmt(FLANGE_T), outside="right")
    es.dim_h(0, u_f, rf, rf, SEG_ROW, _fmt(BODY_LEN), base_v=rf)
    es.dim_h(0, u_b, rf, rf, OVERALL_ROW, _fmt(TOTAL_LEN), base_v=rf)
    es.dim_h(DOWEL_U, u_f, rb + DOWEL_PROTRUSION, rb + DOWEL_PROTRUSION, DOWEL_Z_ROW, _fmt(DOWEL_Z), outside="left")
    es.dim_h(CROSS_U, u_f, -rb, -rb, CROSS_DIM_OFFSET, _fmt(CROSS_Z))
    es.dim_v(0, 0, -rb, rb, BODY_DIM_OFFSET, f"Ø{_fmt(BODY_D)} {TOL_BODY}")
    es.dim_v(u_f, u_f, -rf, rf, (FLANGE_DIM_U - u_f) * SCALE, f"Ø{_fmt(FLANGE_D)}")
    # datum A on the body, datum B and the perpendicularity frame on the target-side face
    sh.datum_feature(es.P(DATUM_A_U, -rb), (0.0, -1.0), DATUM_AXIS)
    sh.datum_feature(es.P(u_b, DATUM_B_V), (1.0, 0.0), DATUM_FACE)
    fb = sh.fcf(*es.P(*FRAME_B_POS), "perp", f"{TOL_PERP_FLANGE} / Ø{_fmt(PERP_DIA)}", (DATUM_AXIS,))
    sh.frame_leader(fb["right"], es.P(u_b, FRAME_B_TARGET_V))
    # callouts, text column right of the section
    def right(target: tuple[float, float], text: str, y_paper: float) -> None:
        tp = es.P(*target)
        es.leader(target, text, SIDE_CALLOUT_X - 5.0 - tp[0], y_paper - tp[1])
    right((DOWEL_U + dh, rb + DOWEL_PROTRUSION / 2),
          f"DOWEL PIN Ø{_fmt(DOWEL_D)} {TOL_PIN} × {_fmt(DOWEL_PIN_LEN)}, {_fmt(DOWEL_PROTRUSION)} PROUD,\n"
          f"IN RADIAL HOLE Ø{_fmt(DOWEL_D)} {TOL_DOWEL_HOLE} × {_fmt(DOWEL_HOLE_DEPTH)} AT +Ym", SIDE_CALLOUT_Y["dowel"])
    right((CROSS_U + CROSS_D / 2, 0.0), f"CROSS HOLE Ø{_fmt(CROSS_D)} THRU, ALONG x,\nFOR THE Ø8 BALL-LOCK PIN",
          SIDE_CALLOUT_Y["cross"])
    right((u_f + MOUNT_CBORE_DEPTH / 2, -vh + rcb),
          f"HOLE AT 225° ROTATED INTO\nTHE SECTION: Ø{_fmt(MOUNT_CLEAR_D)} THRU,\n"
          f"CBORE Ø{_fmt(MOUNT_CBORE_D)} × {_fmt(MOUNT_CBORE_DEPTH)} DEEP", SIDE_CALLOUT_Y["holes"])

    # =====================================================================
    # DETAIL M: MATING PATTERN ON EACH TARGET BACK PLATE (seen from the back of the plate)
    # =====================================================================
    dv = View(sh, DETAIL_CENTER, SCALE)
    dv.circle((0, 0), rf, "phantom")  # flange outline for reference
    dv.circle((0, 0), rm, "center")
    dv.center_h(-rf - END_CENTER_EXT, rf + END_CENTER_EXT, 0.0)
    dv.center_v(-rf - END_CENTER_EXT, rf + END_CENTER_EXT, 0.0)
    r_minor = thread_minor_diameter(MOUNT_THREAD_D, MOUNT_THREAD_PITCH) / 2
    holes = mount_hole_positions()
    for (x, y) in holes:
        dv.circle((x, y), r_minor, "outline")
        dv.circle((x, y), MOUNT_THREAD_D / 2, "thin", 0.0, THREAD_ARC_END_DEG)
        dv.center_cross((x, y), MOUNT_THREAD_D / 2)
    # dowel direction key (red arrow along +Ym)
    a0 = dv.P(0.0, rf + END_CENTER_EXT + KEY_ARROW_FROM)
    a1 = dv.P(0.0, rf + END_CENTER_EXT + KEY_ARROW_FROM + KEY_ARROW_LEN)
    sh.line([a0, a1], "outline", RED)
    sh.arrow(a1, (0, 1), RED)
    sh.text(a1[0] + KEY_TEXT_DX, a1[1] + KEY_TEXT_DY, DOWEL_DIRECTION, size=d.FONT_NOTE, color=RED, weight="bold")
    dv.dim_angle((0, 0), a_hole, 90.0, ARC_DIM_R, "45°", text_dx=ARC_TEXT[0], text_dy=ARC_TEXT[1])
    sh.line([dv.P(*polar((0, 0), ARC_EXT_FROM, a_hole)), dv.P(*polar((0, 0), ARC_EXT_TO, a_hole))], "thin")
    lower_left = holes[2]  # hole at 225 degrees from +Ym
    tp = dv.P(*polar(lower_left, MOUNT_THREAD_D / 2, DETAIL_TAP_ANGLE))
    dbox = dv.leader_circle(lower_left, MOUNT_THREAD_D / 2, DETAIL_TAP_ANGLE,
                            f"4 × M{_fmt(MOUNT_THREAD_D)} THROUGH THE {_fmt(BACK_PLATE_T)} PLATE\n"
                            f"(TAP DRILL Ø{_fmt(MOUNT_TAP_DRILL_D)}), ENGAGED {_fmt(MOUNT_ENGAGEMENT)}",
                            DETAIL_TEXT_END_X + 5.0 - tp[0], DETAIL_CALLOUT_Y["tap"] - tp[1])
    sh.fcf(dbox[2] - 6.0 * 5.0 - 2.0, dbox[1] + FRAME_DY, "position", f"Ø{_fmt(MOUNT_TAP_POSITION_TOL)}", ())
    pp = dv.P(*polar((0, 0), rm, DETAIL_PCD_ANGLE))
    dv.leader_circle((0, 0), rm, DETAIL_PCD_ANGLE, f"Ø{_fmt(MOUNT_PCD)} PCD (BASIC), CENTERED ON\nTHE PLATE AXIS; FLANGE Ø{_fmt(FLANGE_D)} PHANTOM",
                     DETAIL_TEXT_END_X + 5.0 - pp[0], DETAIL_CALLOUT_Y["pcd"] - pp[1])

    # ---- notes, labels, title block --------------------------------------------------
    sh.boxed_note(ROLL_NOTE_POS[0], ROLL_NOTE_POS[1], ROLL_NOTE_W, ROLL_NOTE_TEXT, color=d.BLACK)
    sh.text(END_LABEL[0], END_LABEL[1], "END VIEW FROM THE TIP   SCALE 1:1", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(SIDE_LABEL[0], SIDE_LABEL[1], "SECTION A-A (ALIGNED)   SCALE 1:1", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(DETAIL_LABEL[0], DETAIL_LABEL[1], "DETAIL M: MATING PATTERN ON EACH", size=d.FONT_LABEL, ha="center",
            weight="bold")
    sh.text(DETAIL_LABEL[0], DETAIL_LABEL[1] + DETAIL_LABEL2_DY, "TARGET BACK PLATE   SCALE 1:1", size=d.FONT_LABEL, ha="center",
            weight="bold")
    sh.notes_columns(NOTES_X, NOTES_TOP, NOTES_COLUMN_W, NOTES_COLUMN_GAP, _notes(), NOTES_COLUMNS, size=NOTES_SIZE)
    sh.title_block(TitleInfo(NUMBER, PART_NAME, QUANTITY, MATERIAL, FINISH, SCALE_TEXT))
    return sh.save(os.path.join(out_dir, FILE_NAME))


def _notes() -> list[str]:
    """Numbered notes of the sheet, including the drafter's choices."""
    web = FLANGE_T - MOUNT_CBORE_DEPTH
    return [
        f"Turn from Ø{_fmt(BAR_D)} bar; grind the body to Ø{_fmt(BODY_D)} {TOL_BODY} after turning. Datum A = body axis. "
        f"The flange is larger than the body, so the body cannot come from ground bar.",
        f"Datum B = target-side flange face: flat and perpendicular to A within {TOL_PERP_FLANGE} over Ø{_fmt(PERP_DIA)}. "
        f"Body-side flange face parallel to B within {TOL_PAR_FLANGE}. Machine both faces and the body in one setup.",
        f"Body {_fmt(BODY_LEN)} long from the flange face: it bottoms {_fmt(BORE_GAP)} short of the {_fmt(adapter.BORE_DEPTH)} deep "
        f"bore of PT-01; the flange face seats on the adapter front face.",
        f"Cross hole Ø{_fmt(CROSS_D)} through the body along x, {_fmt(CROSS_Z)} from the flange face (equal to the PT-01 "
        f"cross hole depth), square to the axis; deburr. Ball-lock pin Ø8 passes through the Ø8 H7 and Ø9 holes of PT-01.",
        f"Dowel pin Ø{_fmt(DOWEL_D)} {TOL_PIN} × {_fmt(DOWEL_PIN_LEN)} pressed into a radial hole Ø{_fmt(DOWEL_D)} "
        f"{TOL_DOWEL_HOLE} × {_fmt(DOWEL_HOLE_DEPTH)}, {_fmt(DOWEL_Z)} from the flange face, {_fmt(DOWEL_PROTRUSION)} "
        f"proud at +Ym. PT-01 keyway is {_fmt(adapter.KEY_DEPTH)} deep: radial clearance {KEY_CLEARANCE:.1f}.",
        f"Screws: 4 × M{_fmt(MOUNT_THREAD_D)} × {MOUNT_SCREW_LEN} DIN 7984. Length = flange {_fmt(FLANGE_T)} - counterbore "
        f"{_fmt(MOUNT_CBORE_DEPTH)} + engagement {_fmt(MOUNT_ENGAGEMENT)} = {_fmt(MOUNT_SCREW_LEN_CALC)}. Head "
        f"{_fmt(MOUNT_HEAD_H)} high sits {_fmt(MOUNT_CBORE_RECESS)} below the flange face; web under the head {_fmt(web)}.",
        f"Mating pattern (detail M): 4 × M{_fmt(MOUNT_THREAD_D)} tapped through the {_fmt(BACK_PLATE_T)} aluminum back plate; "
        f"the screw tip stops {_fmt(MOUNT_TIP_SHORT)} short of its far face. Angles are measured from the dowel direction +Ym.",
        "Section A-A is aligned: the cutting plane runs down the vertical axis, then bends toward 225°; the hole at 225° "
        "is rotated into the plane. The cross hole is cut across and appears as a circle.",
        f"Finish: passivate, matte; leave the Ø{_fmt(BODY_D)} {TOL_BODY} body and the faces uncoated. Mark a witness line across the "
        f"flange and the target back plate after tightening.",
        f"Chosen, confirm with the shop: flange Ø{_fmt(FLANGE_D)} × {_fmt(FLANGE_T)}, bar Ø{_fmt(BAR_D)}, dowel {_fmt(DOWEL_PIN_LEN)} long "
        f"{_fmt(DOWEL_Z)} from the face, hole depth {_fmt(DOWEL_HOLE_DEPTH)}, M5 × {MOUNT_SCREW_LEN}, counterbore depth, material grade.",
    ]


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    issues = build(here)
    print(f"{NUMBER}: {len(issues)} layout problem(s)")
    for i in issues:
        print("   ", i)
