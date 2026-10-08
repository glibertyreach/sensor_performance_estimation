"""Shop drawing PT-03: standoffs between the front plate and the back plate of a target.

A standoff is a Ø10 aluminum cylinder with an M5 male stud at each end.  Its body length L is exactly the
gap G between the plates (15 or 60 mm), so the gap is set by the body length alone.  Three targets (T3a, T3b,
T5) each take four standoffs of each length: 3 x 4 = 12 of each length, plus 2 spares of each length = 14 of
each, 28 in all.

Elevations of both lengths and the end view are drawn at 2:1.  Constants a sister script (the plate drawings)
needs are in the first block: STANDOFF_D, STUD_THREAD_D, STUD_PITCH, STUD_LEN, STANDOFF_LENGTHS and
``stud_protrusion``.
"""

from __future__ import annotations

import os

import drafting as d
from drafting import RED, Sheet, TitleInfo, View, polar, thread_minor_diameter

# ---------------------------------------------------------------------------
# Constants exported to the plate drawings
# ---------------------------------------------------------------------------
STANDOFF_D = 10.0  # body diameter
STUD_THREAD_D = 5.0  # M5 male stud at both ends
STUD_PITCH = 0.8  # coarse pitch of M5
STUD_LEN = 8.0  # stud length beyond the shoulder, both ends
STANDOFF_LENGTHS = (15.0, 60.0)  # body lengths = gaps G between the plates
LENGTH_TOL = 0.05  # tolerance of the body length (+- value)
SQUARE_TOL = 0.02  # shoulder faces square to the axis
MATCH_TOL = 0.02  # lengths of the four standoffs of one group agree within this
TARGET_GROUPS = ("T3a", "T3b", "T5")  # targets that take a set
PER_GROUP = 4  # standoffs of each length per target
SPARES_PER_LENGTH = 2  # spares of each length
FRONT_PLATE_T = 6.0  # front plate thickness (given)
BACK_PLATE_T = 8.0  # back plate thickness (given)


def stud_protrusion(plate_t: float) -> float:
    """How far a stud of STUD_LEN sticks out of a through-tapped plate of thickness ``plate_t`` (0 if none)."""
    return max(0.0, STUD_LEN - plate_t)


# ---------------------------------------------------------------------------
# Part dimensions (mm)
# ---------------------------------------------------------------------------
NUMBER = "PT-03"  # drawing number
PART_NAME = "Standoff set"  # part name in the title block
FILE_NAME = "PT-03_standoff_set.png"  # output file
QTY_PER_LENGTH = len(TARGET_GROUPS) * PER_GROUP + SPARES_PER_LENGTH  # 14 of each length
QTY_TOTAL = QTY_PER_LENGTH * len(STANDOFF_LENGTHS)  # 28 in all
QUANTITY = f"{QTY_TOTAL} ({QTY_PER_LENGTH} + {QTY_PER_LENGTH})"
BODY_CHAMFER = 0.5  # chamfer 0.5 x 45 on the outer edge of the body at both shoulders
STUD_CHAMFER = 0.5  # lead chamfer 0.5 x 45 at the end of each stud
BAR_D = 12.0  # stock bar diameter (chosen)
TOL_THREAD = "6g"  # external thread tolerance class (chosen, ISO default)
RUNOUT_MAX = STUD_PITCH  # thread runout at the shoulder allowed up to one pitch (chosen)
DATUM_AXIS = "A"  # body axis
MATERIAL = "Aluminum 6061-T6"
FINISH = "Bead blast, matte, to match the plates"

# derived
BODY_R = STANDOFF_D / 2  # body radius
STUD_R = STUD_THREAD_D / 2  # stud major radius
STUD_MINOR_R = thread_minor_diameter(STUD_THREAD_D, STUD_PITCH) / 2  # stud minor radius
SHOULDER_FLAT = BODY_R - BODY_CHAMFER - STUD_R  # flat width of the shoulder face between stud and chamfer


def overall_length(length: float) -> float:
    """Overall length including both studs (reference only)."""
    return length + 2 * STUD_LEN


# ---------------------------------------------------------------------------
# Sheet layout (paper mm)
# ---------------------------------------------------------------------------
SCALE = 2.0  # all views 2:1
SCALE_TEXT = "2:1"
END_CENTER = (252.0, 124.0)  # paper position of the axis in the end view
ELEV_U0_X = 101.0  # paper x of the left shoulder face (u = 0) in both elevations
ELEV_Y = {60.0: 215.0, 15.0: 125.0}  # paper y of the axis in the elevation of each length
END_LABEL = (252.0, 160.0)  # center and baseline of the end view label (first line)
LABEL_DY = -5.0  # second label line offset
ELEV_LABEL = (160.0, 253.0)  # center and baseline of the elevation label
SUBLABEL_X = 14.0  # elevation sub-label left edge (paper)
SUBLABEL_DY = 1.0  # sub-label first line baseline above the axis (paper)
SUBLABEL_LINE = 5.5  # distance between the two sub-label lines (paper)
ROW_1 = 9.0  # first dimension row above the body (paper)
ROW_2 = 17.0  # second dimension row above the body (paper)
DIA_TEXT_POS = 0.2  # position of the body diameter text along its dimension line
DIA_DIM_OFFSET = 10.0  # body diameter dimension offset right of the right stud (paper)
CENTER_EXT = 3.0  # axis extends this far past the stud tips (model)
THREAD_LEADER = (8.0, -14.0)  # offset of the thread callout from the stud on the short standoff (paper)
CHAMFER_LEADER = (14.0, -22.0)  # offset of the body chamfer callout on the long standoff (paper)
FRAME_DY = -18.0  # perpendicularity frames this far below the shoulder faces (paper)
FRAME_LEFT_DX = -36.0  # left frame left edge offset from the left shoulder (paper)
FRAME_RIGHT_DX = -36.0  # right frame left edge offset from the right shoulder (paper)
DATUM_U_FRAC = 0.5  # datum A on the body at this fraction of the length
END_LEADERS = {"body": (225.0, -12.0, -10.0), "thread": (40.0, 8.0, 12.0)}  # angle, dx, dy in the end view
TABLE_POS = (270.0, 244.0)  # left x and top y of the length table (paper)
TABLE_WIDTHS = (26.0, 22.0, 30.0, 62.0)  # column widths: length, overall, quantity, groups
TABLE_FONT = 9.5  # table text size
NOTES_X = 12.0  # notes block left edge
NOTES_TOP = 99.0  # notes block top
NOTES_COLUMN_W = 72.0  # width of each notes column
NOTES_COLUMN_GAP = 4.0  # gap between notes columns
NOTES_COLUMNS = 3  # number of notes columns
NOTES_SIZE = 9.0  # notes text size (the minimum allowed)
BOX_NOTE_POS = (245.0, 84.0)  # top-left of the boxed gap note
BOX_NOTE_W = 162.0  # width of the boxed gap note
BOX_NOTE_TEXT = ("The body length is the gap G between the plates. The four standoffs of one set are matched in "
                 "length within 0.02; the measured length is recorded as the gap in the as-built record.")
THREAD_ARC_END_DEG = 270.0  # thread minor circle drawn as a three-quarter arc


def _fmt(x: float) -> str:
    """Format a number without trailing zeros."""
    return f"{x:g}"


def _elevation(sh: Sheet, length: float) -> None:
    """Draw the elevation of the standoff of body length ``length`` (axis horizontal, u from the left shoulder)."""
    ev = View(sh, (ELEV_U0_X, ELEV_Y[length]), SCALE)
    rb, rs, rm = BODY_R, STUD_R, STUD_MINOR_R
    cb, cs = BODY_CHAMFER, STUD_CHAMFER
    u_l, u_r = -STUD_LEN, length + STUD_LEN  # stud tips
    upper = [(u_l, rs - cs), (u_l + cs, rs), (0.0, rs), (0.0, rb - cb), (cb, rb), (length - cb, rb),
             (length, rb - cb), (length, rs), (u_r - cs, rs), (u_r, rs - cs)]
    ev.polyline(upper, "outline")
    ev.polyline([(u, -v) for u, v in upper], "outline")
    ev.line(upper[0], (u_l, -(rs - cs)), "outline")
    ev.line(upper[-1], (u_r, -(rs - cs)), "outline")
    ev.line((0.0, rs), (0.0, -rs), "thin")  # shoulder edge seen through (the stud root)
    ev.line((length, rs), (length, -rs), "thin")
    for sv in (1, -1):  # thread minor diameter, thin
        ev.line((u_l + cs, sv * rm), (0.0, sv * rm), "thin")
        ev.line((length, sv * rm), (u_r - cs, sv * rm), "thin")
    ev.center_h(u_l - CENTER_EXT, u_r + CENTER_EXT, 0.0)
    # dimensions above the body
    ev.dim_h(u_l, 0.0, rb, rb, ROW_1, _fmt(STUD_LEN), base_v=rb)
    ev.dim_h(0.0, length, rb, rb, ROW_1, f"{_fmt(length)} ±{_fmt(LENGTH_TOL)}", base_v=rb)
    ev.dim_h(length, u_r, rb, rb, ROW_1, _fmt(STUD_LEN), base_v=rb)
    ev.dim_h(u_l, u_r, rb, rb, ROW_2, f"({_fmt(overall_length(length))} OVERALL, REFERENCE)", base_v=rb)
    ev.dim_v(u_r, u_r, -rb, rb, DIA_DIM_OFFSET + 0.0, f"Ø{_fmt(STANDOFF_D)}", base_u=u_r, text_pos=DIA_TEXT_POS)
    sh.text(SUBLABEL_X, ELEV_Y[length] + SUBLABEL_DY + SUBLABEL_LINE, "STANDOFF", size=d.FONT_NOTE, weight="bold")
    sh.text(SUBLABEL_X, ELEV_Y[length] + SUBLABEL_DY, f"L = {_fmt(length)}", size=d.FONT_NOTE, weight="bold")
    if length == min(STANDOFF_LENGTHS):  # thread callout once, on the short standoff
        ev.leader((u_r - STUD_LEN / 2, -rs), f"M{_fmt(STUD_THREAD_D)} × {_fmt(STUD_PITCH)} - {TOL_THREAD} × {_fmt(STUD_LEN)},\n"
                  f"{_fmt(cs)} × 45° LEAD CHAMFER", *THREAD_LEADER)
    if length == max(STANDOFF_LENGTHS):  # chamfer callout and frames once, on the long standoff
        ev.leader((length - cb / 2, -rb + cb / 2), f"{_fmt(cb)} × 45° CHAMFER, BOTH SHOULDERS", *CHAMFER_LEADER)
        sh.datum_feature(ev.P(length * DATUM_U_FRAC, -rb), (0.0, -1.0), DATUM_AXIS)
        # perpendicularity of each shoulder face to the body axis
        for u_face, dx in ((0.0, FRAME_LEFT_DX), (length, FRAME_RIGHT_DX)):
            px, py = ev.P(u_face, -(rs + rb - cb) / 2)
            fr = sh.fcf(px + dx, py + FRAME_DY, "perp", _fmt(SQUARE_TOL), (DATUM_AXIS,))
            sh.frame_leader(fr["right"], (px, py))


def check_shared_constants() -> None:
    """Compare the stud and body constants with ``spigot_pattern.py`` (used by the plate drawings); raise on a mismatch."""
    try:
        import spigot_pattern as sp
    except ImportError:  # the plate drawings are not present: nothing to compare
        return
    if abs(STUD_LEN - sp.STANDOFF_STUD_LENGTH_MM) > 1e-9 or abs(STANDOFF_D - sp.STANDOFF_BODY_DIAMETER_MM) > 1e-9 \
            or abs(STUD_PITCH - sp.M5_PITCH_MM) > 1e-9 or f"M{STUD_THREAD_D:g}" != sp.STANDOFF_STUD:
        raise ValueError("standoff constants differ from spigot_pattern.py")


def build(out_dir: str) -> list[str]:
    """Draw PT-03 and save it.  Returns the QA issues found."""
    check_shared_constants()
    print(f"  [{NUMBER}] stud {_fmt(STUD_LEN)} in the {_fmt(BACK_PLATE_T)} back plate: protrusion {stud_protrusion(BACK_PLATE_T):g}; "
          f"in the {_fmt(FRONT_PLATE_T)} front plate: protrusion {stud_protrusion(FRONT_PLATE_T):g}")
    print(f"  [{NUMBER}] quantity {QTY_PER_LENGTH} of each length, {QTY_TOTAL} in all; shoulder flat {SHOULDER_FLAT:.2f}")
    sh = Sheet(NUMBER)
    sh.border()

    # ---- end view (from the right stud tip) ---------------------------------------
    ev = View(sh, END_CENTER, SCALE)
    ev.circle((0, 0), BODY_R, "outline")
    ev.circle((0, 0), BODY_R - BODY_CHAMFER, "thin")
    ev.circle((0, 0), STUD_R, "outline")
    ev.circle((0, 0), STUD_MINOR_R, "thin", 0.0, THREAD_ARC_END_DEG)
    ext = BODY_R + CENTER_EXT
    ev.center_h(-ext, ext, 0.0)
    ev.center_v(-ext, ext, 0.0)
    ang, dx, dy = END_LEADERS["body"]
    ev.leader_circle((0, 0), BODY_R, ang, f"Ø{_fmt(STANDOFF_D)}", dx, dy)
    ang, dx, dy = END_LEADERS["thread"]
    ev.leader_circle((0, 0), STUD_R, ang, f"M{_fmt(STUD_THREAD_D)} - {TOL_THREAD}", dx, dy)

    # ---- elevations ---------------------------------------------------------------
    for length in STANDOFF_LENGTHS:
        _elevation(sh, length)

    # ---- table ------------------------------------------------------------------------
    rows = [("BODY L", "OVERALL", "QUANTITY", "MATCHED GROUPS OF 4")]
    for length in STANDOFF_LENGTHS:
        rows.append((f"{_fmt(length)} ±{_fmt(LENGTH_TOL)}", _fmt(overall_length(length)), f"{QTY_PER_LENGTH}",
                     " + ".join(f"{g}" for g in TARGET_GROUPS) + f" + {SPARES_PER_LENGTH} spare"))
    sh.table(TABLE_POS[0], TABLE_POS[1], TABLE_WIDTHS, rows, "STANDOFF LENGTHS (G = L)", size=TABLE_FONT)

    # ---- labels, notes, title block ---------------------------------------------------
    sh.boxed_note(BOX_NOTE_POS[0], BOX_NOTE_POS[1], BOX_NOTE_W, BOX_NOTE_TEXT, color=d.BLACK)
    sh.text(END_LABEL[0], END_LABEL[1], "END VIEW", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(END_LABEL[0], END_LABEL[1] + LABEL_DY, f"SCALE {SCALE_TEXT}", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(ELEV_LABEL[0], ELEV_LABEL[1], f"ELEVATIONS, BOTH LENGTHS   SCALE {SCALE_TEXT}", size=d.FONT_LABEL, ha="center",
            weight="bold")
    sh.notes_columns(NOTES_X, NOTES_TOP, NOTES_COLUMN_W, NOTES_COLUMN_GAP, _notes(), NOTES_COLUMNS, size=NOTES_SIZE)
    sh.title_block(TitleInfo(NUMBER, PART_NAME, QUANTITY, MATERIAL, FINISH, SCALE_TEXT))
    return sh.save(os.path.join(out_dir, FILE_NAME))


def _notes() -> list[str]:
    """Numbered notes of the sheet, including the drafter's choices."""
    n_groups = len(TARGET_GROUPS)
    return [
        f"Turn from Ø{_fmt(BAR_D)} bar. Datum A = body axis. Body length L is measured between the two shoulder faces, "
        f"{_fmt(STANDOFF_LENGTHS[0])} or {_fmt(STANDOFF_LENGTHS[1])} ±{_fmt(LENGTH_TOL)}; shoulder faces square to A "
        f"within {_fmt(SQUARE_TOL)} and flat. Face both shoulders in one setup.",
        f"Studs M{_fmt(STUD_THREAD_D)} × {_fmt(STUD_PITCH)} - {TOL_THREAD}, {_fmt(STUD_LEN)} long at both ends, cut to the "
        f"shoulder; thread runout up to {_fmt(RUNOUT_MAX)} at the shoulder is allowed. {_fmt(STUD_CHAMFER)} × 45° lead "
        f"chamfer on each stud; {_fmt(BODY_CHAMFER)} × 45° chamfer on the body edge at both shoulders.",
        f"Groups: {n_groups} targets ({', '.join(TARGET_GROUPS)}) × {PER_GROUP} of each length = {n_groups * PER_GROUP} of "
        f"each length, plus {SPARES_PER_LENGTH} spares of each length = {QTY_PER_LENGTH} of each, {QTY_TOTAL} in all.",
        f"The {PER_GROUP} standoffs of one group (same length, same target) are matched in length within {_fmt(MATCH_TOL)}, "
        f"so that the gap G is uniform. Measure every standoff after finishing, mark group and serial number on the body, "
        f"and record the measured length as the gap in the as-built record.",
        f"Studs screw into tapped holes in the {_fmt(FRONT_PLATE_T)} mm front plate (or the {_fmt(FRONT_PLATE_T)} mm raised "
        f"square) and the {_fmt(BACK_PLATE_T)} mm back plate. An {_fmt(STUD_LEN)} mm stud flush in the {_fmt(BACK_PLATE_T)} mm "
        f"plate does not protrude; in the {_fmt(FRONT_PLATE_T)} mm plate it would protrude {stud_protrusion(FRONT_PLATE_T):g} mm. "
        f"Chosen as given; confirm with the plate drawings (shorter stud at that end if needed).",
        "Finish: bead blast to match the plates, then measure. Dimensions apply after finishing; keep the shoulder faces "
        "flat (light blast or masked) and do not round the edges.",
        f"Chosen, confirm with the shop: Ø{_fmt(STANDOFF_D)} body, Ø{_fmt(BAR_D)} bar, {TOL_THREAD} thread class, runout, "
        f"{_fmt(BODY_CHAMFER)} × 45° chamfers, the split of the spares ({SPARES_PER_LENGTH} + {SPARES_PER_LENGTH}).",
    ]


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    issues = build(here)
    print(f"{NUMBER}: {len(issues)} layout problem(s)")
    for i in issues:
        print("   ", i)
