"""Shop drawing PT-06 (two sheets): T4 disk plate.

Sheet 1: the back plate (336 x 198 x 8, spigot pattern in its back face) with
the blind post holes of the three disk sites and the post-only site, the table of
site positions, and a section through the largest disk.  Sheet 2: the disks with
their posts (one assembly per disk per gap) and the post-only posts.

Coordinates
-----------
Plan: target frame, origin at the plate center, x right, y up, seen from the
sensor.  Sections: u = x (horizontal) and z (vertical, toward the sensor), z = 0
at the front face of the plate.  Site positions and diameters come from
``ts.targets["T4"].features``; everything else is a named constant or computed.

Gap convention used here (see the report): the gap G is the distance from the
plate's front face to the BACK face of the disk, as for the standoffs of the other
targets, so the front face of a disk stands G + (disk thickness) above the plate.
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
T4 = TARGET_SET.targets["T4"]
FEATURES = {f.site_id: f for f in T4.features}
GAP_SMALL_MM = PARAMS.gap_small_mm
GAP_LARGE_MM = PARAMS.gap_large_mm
GAPS = (GAP_SMALL_MM, GAP_LARGE_MM)
PLATE_W = c.round_up_mm(2 * T4.half_width_mm)  # 335.94 -> 336
PLATE_H = c.round_up_mm(2 * T4.half_height_mm)  # 197.63 -> 198
PLATE_T = c.BACK_PLATE_THICKNESS_MM  # back plate thickness (8)
POST_DIAMETER_MM = 2.0  # post: stainless drill rod, h6
POST_HOLE_DEPTH_MM = 6.0  # reamed blind hole depth in the plate (from the front face)
POST_HOLE_TOL = "H7"  # post hole and disk bore fit
POST_ROD_TOL = "h6"  # post diameter fit
DISK_THICKNESS_MM = {"small": 2.0, "medium": 3.0, "large": 4.0}  # by ladder level
LEVEL_NAMES = ("small", "medium", "large")  # level_index 0, 1, 2
DISK_BORE_FLOOR_MM = 0.5  # material left in front of the post bore (chosen; keeps the front face solid)
DISK_DIAMETER_TOL_MM = c.SITE_DIAMETER_TOL_MM  # +/-0.02 on the front diameter
POSITION_TOL_MM = c.POSITION_TOL_MM  # +/-0.1 on every site position

DISKS = sorted((f for f in T4.features if f.kind == "disk"), key=lambda f: f.level_index)
POSTS = [f for f in T4.features if f.kind == "post"]
BLANKS = sorted((f for f in T4.features if f.kind == "blank"), key=lambda f: f.site_id)
assert len(DISKS) == 3 and len(POSTS) == 1 and len(BLANKS) == 3, "T4 sites changed: revisit the drawing"
LARGEST = DISKS[-1]  # the section goes through this disk


def disk_geometry(f) -> dict[str, float]:
    """Computed dimensions of the disk for site ``f``."""
    t = DISK_THICKNESS_MM[LEVEL_NAMES[f.level_index]]
    run = c.bevel_run(t)
    back_d = f.diameter_mm - 2 * run
    assert back_d > POST_DIAMETER_MM + 1.0, f"back face of {f.site_id} too small for the post"
    bore = t - DISK_BORE_FLOOR_MM
    return {"t": t, "run": run, "back_d": back_d, "bore": bore,
            **{f"post_{g:g}": g + POST_HOLE_DEPTH_MM + bore for g in GAPS}}


def post_only_length(gap: float) -> float:
    """Post-only post: from the hole bottom to the plane of a disk's back face (gap + hole depth)."""
    return gap + POST_HOLE_DEPTH_MM


# ---------------------------------------------------------------------------
# Sheet 1 layout (paper mm)
# ---------------------------------------------------------------------------
PLAN_SCALE = 0.5  # plan at 1:2
PLAN_CENTER = (106.0, 193.0)  # paper position of the plate center
SEC_SCALE = 1.0  # section at 1:1
SEC_ORIGIN_X = 328.0  # paper x of the disk axis in the section
SEC_ORIGIN_Y = 180.0  # paper y of z = 0 (front face of the plate)
SEC_HALF = 46.0  # section cropped at +/- this from the disk axis
SHOWN_GAP_MM = GAP_SMALL_MM  # section shows the 15 mm set
CALL_X = 198.0  # left end of the plan callout texts
TABLE_X = 200.0  # table left edge
TABLE_TOP = 256.0  # table top
NOTES_X = 12.0
NOTES_TOP = 129.0
NOTES_W = 176.0
DIM_TOP = 7.0
DIM_LEFT = -8.0
LABEL_DY = -9.0
CUT_END_X = 6.0  # cutting-plane ends this far beyond the plate edge (model mm)
ID_LABEL_DY = 1.8  # site id label above its circle (paper)
CROSS_ARM_PAPER = d.CENTER_CROSS_EXT + 1.0  # reach of the center cross of a hole above its center (paper)
LABEL_RIGHT = ("disk_02",)  # sites whose id label goes beside the circle (the spigot circle is above it)

SITE_TITLE = f"SITES (target frame, origin at plate center); POSITION TOL ±{c.fmt(POSITION_TOL_MM)}"


def site_rows() -> list[list[str]]:
    rows = [["Site", "Kind", "x", "y", "Front Ø", "Feature in plate"]]
    for f in DISKS + POSTS + BLANKS:
        if f.kind == "disk":
            dia = f"{c.fmt2(f.diameter_mm)} ±{c.fmt(DISK_DIAMETER_TOL_MM)}"
            feat = f"Ø{c.fmt(POST_DIAMETER_MM)} {POST_HOLE_TOL} × {c.fmt(POST_HOLE_DEPTH_MM)} blind"
        elif f.kind == "post":
            dia = f"{c.fmt(POST_DIAMETER_MM)} (post)"
            feat = f"Ø{c.fmt(POST_DIAMETER_MM)} {POST_HOLE_TOL} × {c.fmt(POST_HOLE_DEPTH_MM)} blind"
        else:
            dia = f"{c.fmt2(f.diameter_mm)} (patch)"
            feat = "none: plain plate"
        rows.append([f.site_id, f.kind, f"{f.x_mm:+.2f}", f"{f.y_mm:+.2f}", dia, feat])
    return rows


def build_sheet1(out_dir: str) -> list[str]:
    sh = Sheet("PT-06 sheet 1")
    sh.border()
    plan = View(sh, PLAN_CENTER, PLAN_SCALE)
    hw, hh = PLATE_W / 2, PLATE_H / 2
    plan.polyline([(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)], "outline", closed=True)
    for f in DISKS:
        plan.circle((f.x_mm, f.y_mm), f.diameter_mm / 2, "thin")  # mating disk, seen from the front
    for f in BLANKS:
        plan.circle((f.x_mm, f.y_mm), f.diameter_mm / 2, "phantom")
    for f in DISKS + POSTS:
        plan.circle((f.x_mm, f.y_mm), POST_DIAMETER_MM / 2, "outline")  # post hole
        plan.center_cross((f.x_mm, f.y_mm), POST_DIAMETER_MM / 2)
    for f in DISKS + POSTS + BLANKS:
        r = (POST_DIAMETER_MM if f.kind == "post" else f.diameter_mm) / 2
        if f.site_id in LABEL_RIGHT:  # label beside the circle instead of above it
            pt = plan.P(f.x_mm + r, f.y_mm)
            sh.text(pt[0] + ID_LABEL_DY, pt[1] - 1.2, f.site_id, size=d.FONT_MIN, ha="left")
        else:
            pt = plan.P(f.x_mm, f.y_mm + r)
            sh.text(pt[0], pt[1] + max(CROSS_ARM_PAPER - r * PLAN_SCALE, 0.0) + ID_LABEL_DY, f.site_id,
                    size=d.FONT_MIN, ha="center")
    c.plan_spigot_pattern(plan)
    plan.center_h(-hw - 5, hw + 5, 0.0)
    plan.center_v(-hh - 5, hh + 5, 0.0)
    # cutting plane through the largest disk
    yc = LARGEST.y_mm
    plan.cutting_plane([(-hw - CUT_END_X, yc), (hw + CUT_END_X, yc)], sight=(0.0, 1.0), label="A")
    plan.dim_h(-hw, hw, hh, hh, DIM_TOP, c.fmt(PLATE_W))
    plan.dim_v(-hw, -hw, -hh, hh, DIM_LEFT, c.fmt(PLATE_H))

    def callout(target, text, slot_y):
        t = plan.P(*target)
        return plan.leader(target, text, CALL_X - t[0] - d.LEADER_SHOULDER - d.LEADER_TEXT_GAP, slot_y - t[1])

    post = POSTS[0]
    callout((post.x_mm + POST_DIAMETER_MM / 2, post.y_mm),
            f"4 × Ø{c.fmt(POST_DIAMETER_MM)} {POST_HOLE_TOL} REAMED, {c.fmt(POST_HOLE_DEPTH_MM)} DEEP, BLIND, FROM THE FRONT\n"
            f"FACE (3 DISK SITES + 1 POST-ONLY SITE); POSITION ±{c.fmt(POSITION_TOL_MM)}", 150.0)
    arrow = c.orientation_arrow()
    callout(arrow[3], f"ORIENTATION ARROW, BACK FACE, POINTS {sp.SPIGOT_DOWEL_DIRECTION}:\n"
                      f"{c.fmt(sp.ORIENTATION_MARK_WIDTH_MM)} WIDE × {c.fmt(sp.ORIENTATION_MARK_DEPTH_MM)} DEEP", 136.0)
    sp_pts = c.spigot_hole_points()
    callout((sp_pts[3][0] + c.M5_MAJOR_MM / 2, sp_pts[3][1]),
            f"4 × {sp.SPIGOT_SCREW} THRU IN THE BACK PLATE (SPIGOT PT-02), Ø{c.fmt(sp.SPIGOT_PCD_MM)} PCD AT\n"
            f"{', '.join(c.fmt(a) for a in sp.SPIGOT_HOLE_ANGLES_DEG)}° FROM +x; Ø{c.fmt(sp.SPIGOT_FLANGE_DIAMETER_MM)} FLANGE (PHANTOM)", 119.0)
    b = BLANKS[1]
    callout((b.x_mm + b.diameter_mm / 2, b.y_mm),
            "BLANK SITE (PHANTOM CIRCLE): NO FEATURE,\nKEEP FREE OF MARKS (3 PLACES)", 92.0)
    sh.text(PLAN_CENTER[0], PLAN_CENTER[1] - hh * PLAN_SCALE + LABEL_DY,
            "FRONT VIEW (FROM THE SENSOR)   SCALE 1:2", size=d.FONT_LABEL, ha="center", weight="bold")

    # ---- table of sites ----
    rows = site_rows()
    sh.table(TABLE_X, TABLE_TOP, [24.0, 14.0, 21.0, 21.0, 34.0, 56.0], rows, SITE_TITLE, size=d.FONT_MIN)

    # ---- section A-A through the largest disk (cropped) ----
    sec = View(sh, (SEC_ORIGIN_X, SEC_ORIGIN_Y), SEC_SCALE)
    g = SHOWN_GAP_MM
    G = disk_geometry(LARGEST)
    t, R, run = G["t"], LARGEST.diameter_mm / 2, G["run"]
    u_l, u_r = -SEC_HALF, SEC_HALF  # relative to the disk axis (section u = x - disk x)
    hole_r = POST_DIAMETER_MM / 2
    # plate with the blind hole in its front face
    plate = [(u_l, -PLATE_T), (u_r, -PLATE_T), (u_r, 0.0), (hole_r, 0.0), (hole_r, -POST_HOLE_DEPTH_MM),
             (-hole_r, -POST_HOLE_DEPTH_MM), (-hole_r, 0.0), (u_l, 0.0)]
    c.region(sec, plate, crop_u=(u_l, u_r))
    c.break_line(sec, u_l, 0.0, -PLATE_T)
    c.break_line(sec, u_r, 0.0, -PLATE_T)
    top = g + t
    bore_top = g + G["bore"]
    disk = [(-R, top), (R, top), (R, top - c.EDGE_LAND_MM), (R - run, g), (hole_r, g), (hole_r, bore_top),
            (-hole_r, bore_top), (-hole_r, g), (-R + run, g), (-R, top - c.EDGE_LAND_MM)]
    c.region(sec, disk)
    post_poly = [(-hole_r, -POST_HOLE_DEPTH_MM), (hole_r, -POST_HOLE_DEPTH_MM), (hole_r, bore_top), (-hole_r, bore_top)]
    c.region(sec, post_poly, other=True)
    sec.center_v(-PLATE_T - 4, top + 3, 0.0)
    # dimensions
    sec.dim_h(-R, R, top, top, 7.0, f"Ø{c.fmt2(LARGEST.diameter_mm)} ±{c.fmt(DISK_DIAMETER_TOL_MM)}")
    sec.dim_v(R, R - run, top, g, 8.0, c.fmt(t))
    sec.dim_v(hole_r, hole_r, 0.0, g, 18.0, c.fmt(g))
    sec.dim_v(u_r, u_r, 0.0, -PLATE_T, 8.0, c.fmt(PLATE_T))
    sec.dim_v(-hole_r, -hole_r, -POST_HOLE_DEPTH_MM, 0.0, -18.0, c.fmt(POST_HOLE_DEPTH_MM))
    sec.leader((-hole_r, g / 2), f"POST \u00d8{c.fmt(POST_DIAMETER_MM)} {POST_ROD_TOL}", -26.0, 4.0, terminator="dot")
    sh.text(SEC_ORIGIN_X, SEC_ORIGIN_Y - PLATE_T * SEC_SCALE - 11.0,
            "SECTION A-A   SCALE 1:1", size=d.FONT_LABEL, ha="center", weight="bold")
    sh.text(SEC_ORIGIN_X, SEC_ORIGIN_Y - PLATE_T * SEC_SCALE - 16.5,
            f"through disk_02 (largest) with its post, {c.fmt(g)} mm set", size=d.FONT_NOTE, ha="center")

    notes = [
        f"Back plate {c.fmt(PLATE_W)} × {c.fmt(PLATE_H)} × {c.fmt(PLATE_T)} (outline rounded up to whole mm from "
        f"{2 * T4.half_width_mm:.2f} × {2 * T4.half_height_mm:.2f}). Site positions and diameters are from the target "
        f"definition (table); position ±{c.fmt(POSITION_TOL_MM)}.",
        f"Post holes Ø{c.fmt(POST_DIAMETER_MM)} {POST_HOLE_TOL} reamed, {c.fmt(POST_HOLE_DEPTH_MM)} deep, blind, from the "
        f"front face, square to it. Posts (sheet 2) are a slip fit, secured with low-strength removable retaining compound; "
        f"swap the posts to change the gap. Thin circles are the disks (sheet 2); phantom circles are blank sites: no "
        f"feature, keep free of marks.",
        c.NOTE_FINISH + " " + c.NOTE_FLATNESS + " Blast the plate face with the disks and posts in the same batch.",
        c.NOTE_SPIGOT,
        f"Gap G ({c.fmt(GAP_SMALL_MM)} or {c.fmt(GAP_LARGE_MM)} mm) is measured from the plate's front face to the back "
        f"face of the disk. The plate carries no standoffs.",
    ]
    bottom = sh.notes_block(NOTES_X, NOTES_TOP, NOTES_W, notes, size=c.NOTE_SIZE)
    assert bottom > d.FRAME_BOTTOM, f"notes overflow the sheet (bottom {bottom:.1f})"
    sh.title_block(TitleInfo("PT-06 sheet 1/2", "T4 disk plate: back plate", "1", c.MATERIAL_TEXT,
                             c.FINISH_TEXT, "1:2, 1:1"))
    path = os.path.join(out_dir, "PT-06_T4_disk_plate_sheet1.png")
    issues = sh.save(path)
    c.report_sheet("PT-06 sheet 1", path, issues)
    return issues


def print_checks() -> None:
    print(f"PT-06 plate {PLATE_W:g} x {PLATE_H:g} x {PLATE_T:g}")
    for f in DISKS:
        G = disk_geometry(f)
        print(f"PT-06 {f.site_id}: front D {f.diameter_mm:.3f}, t {G['t']:g}, back D {G['back_d']:.3f}, bore {G['bore']:g}, "
              f"post lengths {G['post_15']:g} / {G['post_60']:g}")
    for g in GAPS:
        print(f"PT-06 post-only length at gap {g:g}: {post_only_length(g):g}")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    print_checks()
    build_sheet1(here)
