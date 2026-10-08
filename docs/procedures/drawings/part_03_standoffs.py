"""Shop drawing PT-03: standoffs between the front plate and the back plate of a target.

A standoff is a Ø10 aluminum cylinder with an M5 male stud at each end.  The ends differ: the BACK end has an
M5 x 8 stud (into the 8 mm back plate, through-tapped, end flush with the back face); the FRONT end has an
M5 x 5.5 stud (it ends 0.5 below the front face of a 6 mm through-tapped front plate and also seats in the
10 mm raised square of T3a).  A groove on the body next to the front shoulder marks the front end.

The gap G (15 or 60 mm, from CharacterizationParameters) is the depth step between the FRONT face of the front
part and the front face of the back plate.  The standoff body is therefore G minus the front part's
thickness (6 mm front plate of T3b and T5; 10 mm raised square of T3a): 9, 54, 5 and 50 mm.  Each target
takes four standoffs of each of its two lengths; each length also gets 2 spares: 10 + 10 + 6 + 6 = 32 pieces.

Elevations of the longest (54) and the shortest (5) body and the end view are drawn at 2:1; the other
lengths are in the table.  Constants a sister script (the plate drawings)
needs are in the first block: STANDOFF_D, STUD_THREAD_D, STUD_PITCH, STUD_FRONT_LENGTH_MM, STUD_BACK_LENGTH_MM,
STANDOFF_BODY_LENGTHS_MM (dict {(gap_mm, front_thickness_mm): body_mm}), STANDOFF_LENGTHS and
``stud_recess``.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass

# The script must run from any working directory: make the sibling modules and the repository root
# (for ``sensorperf``) importable.
HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
for _p in (HERE, REPO_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import drafting as d  # noqa: E402
import spigot_pattern as _sp  # noqa: E402
from sensorperf.parameters import CharacterizationParameters  # noqa: E402
from drafting import RED, Sheet, TitleInfo, View, polar, thread_minor_diameter  # noqa: E402

# ---------------------------------------------------------------------------
# Constants exported to the plate drawings
# ---------------------------------------------------------------------------
STANDOFF_D = 10.0  # body diameter
STUD_THREAD_D = 5.0  # M5 male stud at both ends
STUD_PITCH = 0.8  # coarse pitch of M5
STUD_BACK_LENGTH_MM = 8.0  # stud length at the BACK end (back plate 8 mm, through-tapped, end flush)
STUD_FRONT_LENGTH_MM = 5.5  # stud length at the FRONT end (ends 0.5 below the front face of a 6 mm plate)
STUD_LEN = STUD_BACK_LENGTH_MM  # kept for older callers: the back stud length
LENGTH_TOL = 0.05  # tolerance of the body length (+- value)
SQUARE_TOL = 0.02  # shoulder faces square to the axis
MATCH_TOL = 0.02  # lengths of the four standoffs of one group agree within this
TARGET_GROUPS = ("T3a", "T3b", "T5")  # targets that take standoffs
PER_GROUP = 4  # standoffs of each length per target
SPARES_PER_LENGTH = 2  # spares of each body length
# thickness of the front part: the standoff body is the gap G minus this (given; taken from spigot_pattern.py if it has them)
FRONT_PLATE_THICKNESS_MM = getattr(_sp, "FRONT_PLATE_THICKNESS_MM", 6.0)  # front plate of T3b and T5, through-tapped
RAISED_SQUARE_THICKNESS_MM = getattr(_sp, "RAISED_SQUARE_THICKNESS_MM", 10.0)  # raised square of T3a, blind holes
FRONT_PLATE_T = FRONT_PLATE_THICKNESS_MM  # short names used below
RAISED_SQUARE_T = RAISED_SQUARE_THICKNESS_MM
FRONT_THICKNESS = {"T3a": RAISED_SQUARE_THICKNESS_MM, "T3b": FRONT_PLATE_THICKNESS_MM, "T5": FRONT_PLATE_THICKNESS_MM}
_PARAMS = CharacterizationParameters()
GAPS_MM = (_PARAMS.gap_small_mm, _PARAMS.gap_large_mm)  # gaps G: depth step between the two faces the sensor sees
GROOVE_W = 0.5  # width of the front-end marker groove
GROOVE_DEPTH = 0.3  # depth of the marker groove
GROOVE_FROM_SHOULDER = 1.0  # distance from the front shoulder face to the near edge of the groove (chosen)
BACK_PLATE_T = 8.0  # back plate thickness (given)


def stud_recess(plate_t: float, stud_len: float) -> float:
    """How far the end of a stud of length ``stud_len`` lies below the far face of a through-tapped plate of
    thickness ``plate_t`` (positive: recessed, 0: flush, negative: it protrudes)."""
    return plate_t - stud_len


@dataclass(frozen=True)
class Variant:
    """One standoff body length: the gap, the thickness of the front part it carries, the targets using it."""

    gap: float  # gap G, mm
    front_t: float  # thickness of the front part, mm
    targets: tuple  # targets that take this length

    @property
    def body(self) -> float:
        """Body length: the gap minus the front part's thickness."""
        return self.gap - self.front_t

    @property
    def qty(self) -> int:
        """Pieces to make: four per target plus the spares."""
        return len(self.targets) * PER_GROUP + SPARES_PER_LENGTH


def _build_variants() -> tuple:
    """All (gap, front thickness) combinations in use, shortest body first."""
    thicknesses = sorted(set(FRONT_THICKNESS.values()), reverse=True)  # thick first: shortest bodies first
    out = []
    for gap in GAPS_MM:
        for t in thicknesses:
            targets = tuple(k for k in TARGET_GROUPS if FRONT_THICKNESS[k] == t)
            out.append(Variant(gap, t, targets))
    return tuple(sorted(out, key=lambda v: v.body))


VARIANTS = _build_variants()  # 5, 9, 50, 54
STANDOFF_BODY_LENGTHS_MM = {(v.gap, v.front_t): v.body for v in VARIANTS}  # for the plate scripts
STANDOFF_LENGTHS = tuple(v.body for v in VARIANTS)  # body lengths, shortest first


# ---------------------------------------------------------------------------
# Part dimensions (mm)
# ---------------------------------------------------------------------------
NUMBER = "PT-03"  # drawing number
PART_NAME = "Standoff set"  # part name in the title block
FILE_NAME = "PT-03_standoff_set.png"  # output file
QTY_TOTAL = sum(v.qty for v in VARIANTS)  # 32 in all
QUANTITY = f"{QTY_TOTAL} ({' + '.join(str(v.qty) for v in sorted(VARIANTS, key=lambda v: -v.body))})"
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
GROOVE_U1 = GROOVE_FROM_SHOULDER + GROOVE_W  # far edge of the groove from the front shoulder
SHOULDER_FLAT = BODY_R - BODY_CHAMFER - STUD_R  # flat width of the shoulder face between stud and chamfer


def overall_length(length: float) -> float:
    """Overall length including both studs (reference only)."""
    return length + STUD_FRONT_LENGTH_MM + STUD_BACK_LENGTH_MM


# ---------------------------------------------------------------------------
# Sheet layout (paper mm)
# ---------------------------------------------------------------------------
SCALE = 2.0  # all views 2:1
SCALE_TEXT = "2:1"
END_CENTER = (252.0, 124.0)  # paper position of the axis in the end view
ELEV_U0_X = 101.0  # paper x of the left shoulder face (u = 0) in both elevations
ELEV_Y = (215.0, 125.0)  # paper y of the axis of the long and the short elevation
END_LABEL = (252.0, 160.0)  # center and baseline of the end view label (first line)
LABEL_DY = -5.0  # second label line offset
ELEV_LABEL = (160.0, 253.0)  # center and baseline of the elevation label
SUBLABEL_X = 14.0  # elevation sub-label left edge (paper)
SUBLABEL_DY = 1.0  # sub-label first line baseline above the axis (paper)
SUBLABEL_LINE = 5.5  # distance between the two sub-label lines (paper)
ROW_1 = 9.0  # first dimension row above the body (paper)
ROW_2 = 17.0  # second dimension row above the body (paper)
ROW_3 = 25.0  # third dimension row above the body (paper), used by very short bodies
MIN_INLINE_BODY_SPAN = 26.0  # shorter body dimensions (paper mm) put their text outside, in the third row
DIA_TEXT_POS = 0.2  # position of the body diameter text along its dimension line
DIA_DIM_OFFSET = 10.0  # body diameter dimension offset right of the right stud (paper)
CENTER_EXT = 3.0  # axis extends this far past the stud tips (model)
GROOVE_LEADER = (-10.0, -8.0)  # offset of the groove callout from the groove (paper)
THREAD_LEADER = (22.0, -12.0)  # offset of the thread callout from the stud on the short standoff (paper)
CHAMFER_LEADER = (14.0, -22.0)  # offset of the body chamfer callout on the long standoff (paper)
FRAME_DY = -18.0  # perpendicularity frames this far below the shoulder faces (paper)
FRAME_LEFT_DX = -36.0  # left frame left edge offset from the left shoulder (paper)
FRAME_RIGHT_DX = -36.0  # right frame left edge offset from the right shoulder (paper)
DATUM_U_FRAC = 0.5  # datum A on the body at this fraction of the length
END_LEADERS = {"body": (225.0, -12.0, -10.0), "thread": (40.0, 8.0, 12.0)}  # angle, dx, dy in the end view
TABLE_POS = (268.0, 244.0)  # left x and top y of the length table (paper)
TABLE_WIDTHS = (22.0, 11.0, 21.0, 12.0, 74.0)  # column widths: body, gap, overall, quantity, for
TABLE_FONT = 9.5  # table text size
NOTES_X = 12.0  # notes block left edge
NOTES_TOP = 99.0  # notes block top
NOTES_COLUMN_W = 72.0  # width of each notes column
NOTES_COLUMN_GAP = 4.0  # gap between notes columns
NOTES_COLUMNS = 3  # number of notes columns
NOTES_SIZE = 9.0  # notes text size (the minimum allowed)
BOX_NOTE_POS = (245.0, 93.0)  # top-left of the boxed gap note
BOX_NOTE_W = 162.0  # width of the boxed gap note
BOX_NOTE_TEXT = ("The body length is G minus the front part's thickness, so that the step the sensor sees is G. "
                 "The four standoffs of one group are matched in length within 0.02; the measured length plus "
                 "the front thickness is recorded as the gap G in the as-built record.")
THREAD_ARC_END_DEG = 270.0  # thread minor circle drawn as a three-quarter arc


def _fmt(x: float) -> str:
    """Format a number without trailing zeros."""
    return f"{x:g}"


def _elevation(sh: Sheet, variant: Variant, y_axis: float, first: bool) -> None:
    """Draw the elevation of the standoff of ``variant`` (axis horizontal, u from the left shoulder).
    ``first`` marks the long one, which also carries the chamfer callout and the tolerance frames."""
    length = variant.body
    ev = View(sh, (ELEV_U0_X, y_axis), SCALE)
    rb, rs, rm = BODY_R, STUD_R, STUD_MINOR_R
    cb, cs = BODY_CHAMFER, STUD_CHAMFER
    u_l, u_r = -STUD_FRONT_LENGTH_MM, length + STUD_BACK_LENGTH_MM  # stud tips (front stud at the left)
    gd = rb - GROOVE_DEPTH  # radius at the bottom of the marker groove
    upper = [(u_l, rs - cs), (u_l + cs, rs), (0.0, rs), (0.0, rb - cb), (cb, rb), (GROOVE_FROM_SHOULDER, rb),
             (GROOVE_FROM_SHOULDER, gd), (GROOVE_U1, gd), (GROOVE_U1, rb), (length - cb, rb),
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
    # dimensions above the body; a very short body puts its dimension (text to the left) in the third row, the overall in the second
    inline = length * SCALE >= MIN_INLINE_BODY_SPAN
    body_row, overall_row = (ROW_1, ROW_2) if inline else (ROW_3, ROW_2)
    ev.dim_h(u_l, 0.0, rb, rb, ROW_1, f"FRONT {_fmt(STUD_FRONT_LENGTH_MM)}", outside="left", base_v=rb)
    ev.dim_h(0.0, length, rb, rb, body_row, f"{_fmt(length)} ±{_fmt(LENGTH_TOL)}", base_v=rb,
             outside=None if inline else "left")
    ev.dim_h(length, u_r, rb, rb, ROW_1, f"BACK {_fmt(STUD_BACK_LENGTH_MM)}", outside="right", base_v=rb)
    ev.dim_h(u_l, u_r, rb, rb, overall_row, f"({_fmt(overall_length(length))} OVERALL, REFERENCE)", base_v=rb)
    ev.dim_v(u_r, u_r, -rb, rb, DIA_DIM_OFFSET + 0.0, f"Ø{_fmt(STANDOFF_D)}", base_u=u_r, text_pos=DIA_TEXT_POS)
    sh.text(SUBLABEL_X, y_axis + SUBLABEL_DY + SUBLABEL_LINE, "STANDOFF", size=d.FONT_NOTE, weight="bold")
    sh.text(SUBLABEL_X, y_axis + SUBLABEL_DY, f"L = {_fmt(length)}", size=d.FONT_NOTE, weight="bold")
    sh.text(SUBLABEL_X, y_axis + SUBLABEL_DY - SUBLABEL_LINE, f"G {_fmt(variant.gap)}, FRONT {_fmt(variant.front_t)}",
            size=d.FONT_NOTE)
    if not first:  # thread and groove callouts once, on the short standoff
        ev.leader((u_r - STUD_BACK_LENGTH_MM / 2, -rs),
                  f"M{_fmt(STUD_THREAD_D)} × {_fmt(STUD_PITCH)} - {TOL_THREAD} STUDS,\n{_fmt(cs)} × 45° LEAD CHAMFER,\n"
                  f"{_fmt(STUD_BACK_LENGTH_MM)} BACK, {_fmt(STUD_FRONT_LENGTH_MM)} FRONT", *THREAD_LEADER)
        ev.leader(((GROOVE_FROM_SHOULDER + GROOVE_U1) / 2, -gd), f"GROOVE {_fmt(GROOVE_W)} × {_fmt(GROOVE_DEPTH)} DEEP\n"
                  f"MARKS THE FRONT END", *GROOVE_LEADER)
    if first:  # chamfer callout and frames once, on the long standoff
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
    back = getattr(sp, "STANDOFF_STUD_BACK_LENGTH_MM", getattr(sp, "STANDOFF_STUD_LENGTH_MM", STUD_BACK_LENGTH_MM))
    front = getattr(sp, "STANDOFF_STUD_FRONT_LENGTH_MM", STUD_FRONT_LENGTH_MM)  # absent: nothing to compare
    for name, mine in (("FRONT_PLATE_THICKNESS_MM", FRONT_PLATE_THICKNESS_MM),
                       ("RAISED_SQUARE_THICKNESS_MM", RAISED_SQUARE_THICKNESS_MM)):
        if hasattr(sp, name) and abs(getattr(sp, name) - mine) > 1e-9:
            raise ValueError(f"{name} differs from spigot_pattern.py")
    if abs(STUD_BACK_LENGTH_MM - back) > 1e-9 or abs(STUD_FRONT_LENGTH_MM - front) > 1e-9 \
            or abs(STANDOFF_D - sp.STANDOFF_BODY_DIAMETER_MM) > 1e-9 \
            or abs(STUD_PITCH - sp.M5_PITCH_MM) > 1e-9 or f"M{STUD_THREAD_D:g}" != sp.STANDOFF_STUD:
        raise ValueError("standoff constants differ from spigot_pattern.py")


def build(out_dir: str) -> list[str]:
    """Draw PT-03 and save it.  Returns the QA issues found."""
    check_shared_constants()
    print(f"  [{NUMBER}] back stud {_fmt(STUD_BACK_LENGTH_MM)} in the {_fmt(BACK_PLATE_T)} back plate: end "
          f"{stud_recess(BACK_PLATE_T, STUD_BACK_LENGTH_MM):g} below the face; front stud {_fmt(STUD_FRONT_LENGTH_MM)} in the "
          f"{_fmt(FRONT_PLATE_T)} plate: end {stud_recess(FRONT_PLATE_T, STUD_FRONT_LENGTH_MM):g} below the face; "
          f"in the {_fmt(RAISED_SQUARE_T)} raised square: {stud_recess(RAISED_SQUARE_T, STUD_FRONT_LENGTH_MM):g} of "
          f"blind hole left beyond the stud")
    if stud_recess(FRONT_PLATE_T, STUD_FRONT_LENGTH_MM) < 0 or stud_recess(BACK_PLATE_T, STUD_BACK_LENGTH_MM) < 0:
        raise ValueError("a stud protrudes from its plate")
    if STUD_FRONT_LENGTH_MM < STUD_THREAD_D:
        raise ValueError("front stud shorter than one diameter of engagement")
    for v in VARIANTS:
        if v.body <= 0:
            raise ValueError("standoff body length not positive")
        print(f"  [{NUMBER}] G {_fmt(v.gap)} - front {_fmt(v.front_t)} = body {_fmt(v.body)}: {v.qty} pieces "
              f"({', '.join(v.targets)} x {PER_GROUP} + {SPARES_PER_LENGTH} spare)")
    print(f"  [{NUMBER}] {QTY_TOTAL} pieces in all; shoulder flat {SHOULDER_FLAT:.2f}")
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
    _elevation(sh, VARIANTS[-1], ELEV_Y[0], True)  # the longest body (54)
    _elevation(sh, VARIANTS[0], ELEV_Y[1], False)  # the shortest body (5)

    # ---- table ------------------------------------------------------------------------
    rows = [("BODY L", "G", "OVERALL", "QTY", "FOR")]
    for v in sorted(VARIANTS, key=lambda v: -v.body):
        rows.append((f"{_fmt(v.body)} ±{_fmt(LENGTH_TOL)}", _fmt(v.gap), _fmt(overall_length(v.body)), f"{v.qty}",
                     f"{', '.join(v.targets)} (front {_fmt(v.front_t)}, ×{PER_GROUP} + {SPARES_PER_LENGTH} spare)"))
    sh.table(TABLE_POS[0], TABLE_POS[1], TABLE_WIDTHS, rows, "STANDOFF BODIES (L = G - FRONT THICKNESS)", size=TABLE_FONT)

    # ---- labels, notes, title block ---------------------------------------------------
    sh.boxed_note(BOX_NOTE_POS[0], BOX_NOTE_POS[1], BOX_NOTE_W, BOX_NOTE_TEXT, color=d.BLACK)
    sh.text(END_LABEL[0], END_LABEL[1], "END VIEW", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(END_LABEL[0], END_LABEL[1] + LABEL_DY, f"SCALE {SCALE_TEXT}", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(ELEV_LABEL[0], ELEV_LABEL[1], f"ELEVATIONS, LONGEST AND SHORTEST BODY   SCALE {SCALE_TEXT}", size=d.FONT_LABEL, ha="center",
            weight="bold")
    sh.notes_columns(NOTES_X, NOTES_TOP, NOTES_COLUMN_W, NOTES_COLUMN_GAP, _notes(), NOTES_COLUMNS, size=NOTES_SIZE)
    sh.title_block(TitleInfo(NUMBER, PART_NAME, QUANTITY, MATERIAL, FINISH, SCALE_TEXT))
    return sh.save(os.path.join(out_dir, FILE_NAME))


def _notes() -> list[str]:
    """Numbered notes of the sheet, including the drafter's choices."""
    by_len = sorted(VARIANTS, key=lambda v: v.body)
    lengths = ", ".join(_fmt(v.body) for v in by_len)
    groups = "; ".join(f"{' and '.join(v.targets)}: {_fmt(v.body)} (G {_fmt(v.gap)})" for v in by_len)
    qty = ", ".join(f"{_fmt(v.body)} → {v.qty}" for v in by_len)
    return [
        f"Convention: the body length is G minus the front part's thickness, so that the step the sensor sees is G "
        f"(G = {_fmt(GAPS_MM[0])} or {_fmt(GAPS_MM[1])}; front part {_fmt(FRONT_PLATE_THICKNESS_MM)} mm plate of T3b and T5, "
        f"{_fmt(RAISED_SQUARE_THICKNESS_MM)} mm raised square of T3a). Bodies: {lengths} ±{_fmt(LENGTH_TOL)}.",
        f"Turn from Ø{_fmt(BAR_D)} bar. Datum A = body axis. Body length is measured between the two shoulder faces; "
        f"shoulder faces square to A within {_fmt(SQUARE_TOL)} and flat. Face both shoulders in one setup.",
        f"The ends differ. BACK stud M{_fmt(STUD_THREAD_D)} × {_fmt(STUD_PITCH)} - {TOL_THREAD} × {_fmt(STUD_BACK_LENGTH_MM)}; "
        f"FRONT stud M{_fmt(STUD_THREAD_D)} × {_fmt(STUD_PITCH)} - {TOL_THREAD} × {_fmt(STUD_FRONT_LENGTH_MM)}. Cut to the "
        f"shoulder; thread runout up to {_fmt(RUNOUT_MAX)} at the shoulder is allowed. {_fmt(STUD_CHAMFER)} × 45° lead "
        f"chamfer on each stud; {_fmt(BODY_CHAMFER)} × 45° chamfer on the body edge at both shoulders.",
        f"The FRONT end is marked by a groove {_fmt(GROOVE_W)} wide × {_fmt(GROOVE_DEPTH)} deep on the body, its near edge "
        f"{_fmt(GROOVE_FROM_SHOULDER)} from the front shoulder, so that the technician can tell the ends apart. Do not "
        f"swap the ends: the back stud would protrude "
        f"{-stud_recess(FRONT_PLATE_THICKNESS_MM, STUD_BACK_LENGTH_MM):g} through the {_fmt(FRONT_PLATE_THICKNESS_MM)} mm plate.",
        f"Groups of {PER_GROUP}, {SPARES_PER_LENGTH} spares of each length. {groups}. Quantities: {qty}; "
        f"{QTY_TOTAL} in all.",
        f"The {PER_GROUP} standoffs of one group (same length, same target) are matched in length within {_fmt(MATCH_TOL)}, "
        f"so that the step G is uniform. Measure every standoff after finishing, mark group and serial number on the body, "
        f"and record the measured length (G = length + front thickness) in the as-built record.",
        f"BACK stud: through-tapped {_fmt(BACK_PLATE_T)} mm back plate, end flush with the back face. FRONT stud: "
        f"through-tapped {_fmt(FRONT_PLATE_THICKNESS_MM)} mm front plate (T3b, T5), end "
        f"{stud_recess(FRONT_PLATE_THICKNESS_MM, STUD_FRONT_LENGTH_MM):g} below the front face, never protruding; also seats "
        f"in the blind holes of the {_fmt(RAISED_SQUARE_THICKNESS_MM)} mm raised square of T3a. Engagement "
        f"{_fmt(STUD_FRONT_LENGTH_MM)} = {STUD_FRONT_LENGTH_MM / STUD_THREAD_D:.1f} d.",
        "Finish: bead blast to match the plates, then measure. Dimensions apply after finishing; keep the shoulder faces "
        "flat (light blast or masked) and do not round the edges.",
        f"Chosen, confirm with the shop: Ø{_fmt(STANDOFF_D)} body, Ø{_fmt(BAR_D)} bar, {TOL_THREAD} thread class, runout, "
        f"{_fmt(BODY_CHAMFER)} × 45° chamfers, the marker groove, stud lengths {_fmt(STUD_FRONT_LENGTH_MM)} and "
        f"{_fmt(STUD_BACK_LENGTH_MM)}, the convention L = G - front thickness with front parts {_fmt(FRONT_PLATE_THICKNESS_MM)} "
        f"and {_fmt(RAISED_SQUARE_THICKNESS_MM)} mm, spares ({SPARES_PER_LENGTH} per length).",
    ]


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    issues = build(here)
    print(f"{NUMBER}: {len(issues)} layout problem(s)")
    for i in issues:
        print("   ", i)
