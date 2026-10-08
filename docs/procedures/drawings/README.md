# VSX3000 performance testing: shop drawings

Dimensioned shop drawings (millimeters, third-angle projection, general tolerances ISO 2768-mK, date
2026-10-08) for the fixtures of the performance test. Each sheet is a 3300 x 2100 px PNG (200 dpi,
landscape) with its views at one stated scale, a notes block and a title block.

| Drawing | Part | Qty | Material | Scale | File |
|---------|------|-----|----------|-------|------|
| PT-01 | Target adapter (120 x 120 x 30 plate; ISO 9409-1-50-4-M6 flange side; Ø40 H7 bore, keyway, cross hole) | 1 | Aluminum 6061-T6 | 1:1 | `PT-01_target_adapter.png` |
| PT-02 | Target spigot (Ø40 h6 body, Ø80 flange, dowel, cross hole; detail M, mating pattern on each target back plate) | 4 | 303 or 17-4 PH stainless | 1:1 | `PT-02_target_spigot.png` |
| PT-03 | Standoff set (Ø10 body; bodies 5, 9, 50 and 54 = G - front thickness; M5 x 8 back stud, M5 x 5.5 front stud) | 32 (10 + 6 + 10 + 6) | Aluminum 6061-T6 | 2:1 | `PT-03_standoff_set.png` |
| PT-04 | T3a raised square (300 x 300 x 8 back plate with the spigot pattern; 160 x 160 x 10 square, slanted 5 deg, knife edges beveled from the back; four PT-03 standoffs on a 60 mm square) | 1 plate + 1 square | Aluminum tooling plate (MIC-6 or 6061-T6) | 1:2, 1:1, 4:1 | `PT-04_T3a_raised_square.png` |
| PT-05 | T3b square window (300 x 300 x 6 front plate with the 160 mm window, slanted 5 deg, countersunk from the back; 300 x 300 x 8 plain back plate with the spigot pattern; four standoffs at the corners) | 1 front + 1 back plate | Aluminum tooling plate | 1:2, 1:1, 4:1 | `PT-05_T3b_square_window.png` |
| PT-06 | T4 disk plate, two sheets (sheet 1: 336 x 198 x 8 back plate with the post holes and the site table; sheet 2: disks with posts, 6 assemblies, and the 2 post-only posts) | 1 plate; 6 disks + 8 posts | Plate aluminum tooling plate; disks 6061-T6; posts stainless drill rod 2 h6 | 1:2, 1:1; 2:1, 4:1 | `PT-06_T4_disk_plate_sheet1.png`, `PT-06_T4_disk_plate_sheet2.png` |
| PT-07 | T5 cutout plate (336 x 198 x 6 front plate with three countersunk cutouts, 336 x 198 x 8 removable back plate, edge bracket with the spigot extension) | 1 + 1 + 1 | Aluminum tooling plate | 1:2, 1:1, 4:1 | `PT-07_T5_cutout_plate.png` |

PT-01 carries the flange-interface note (ISO 9409-1-50-4-M6, to be confirmed against the chosen robot's
flange drawing before machining). The dowel of PT-02 and the keyway of PT-01 fix the target's roll; the
ball-lock pin fixes its pull-out.

## Regenerate

```
python3 docs/procedures/drawings/make_all.py
```

This needs only `matplotlib` (and `numpy`). It rewrites the PNG files next to the scripts, prints the
geometry checks of each part (hole collisions, stack-ups against the neighboring part) and the result of
an automatic layout check (text against text and against lines, text outside the border, title-block text
outside its cell). The exit status is 1 if any sheet has a layout problem or a geometry check fails.

## Layout of the code

| File | Content |
|------|---------|
| `drafting.py` | Shared drafting helpers (copied from stage 1; extended with title-block cell checks and `View.circle_except`) |
| `part_01_target_adapter.py` | PT-01: plan, section A-A, turned section B-B; `geometry_checks()` |
| `part_02_target_spigot.py` | PT-02: end view, aligned section A-A, detail M; `consistency_checks()` against PT-01 |
| `part_03_standoffs.py` | PT-03: end view, elevations of the longest and shortest body, table of the four bodies (computed from G and the front thickness) |
| `spigot_pattern.py` | Mounting constants shared by the plate drawings (spigot hole pattern, flange diameter, dowel direction, orientation mark, standoff studs); PT-02 and PT-03 compare their own constants with it |
| `target_drawing_common.py` | Shared by PT-04 to PT-07: the specification rules (45 degree back bevel, 0.1 mm land, plate thicknesses, finish), target loading, rounding rule, the 4:1 knife-edge detail, standoff joint sections, hidden-standoff check; `check_against_part_scripts()` cross-checks PT-02 and PT-03 |
| `part_04_t3a_raised_square.py` | PT-04: plan, section A-A through two standoffs (plane parallel to the square's edges), 4:1 knife-edge detail |
| `part_05_t3b_square_window.py` | PT-05: plan, offset section A-A, 4:1 knife-edge detail, hidden-standoff check |
| `part_06_t4_disk_plate.py` | PT-06: sheet 1 (plate, site table, section through the largest disk) and sheet 2 (disk and post assemblies, post-only post, 4:1 disk edge) |
| `part_07_t5_cutout_plate.py` | PT-07: plan, offset section A-A, 4:1 countersink detail, site table, edge bracket (plan and side view), hidden-standoff check |
| `make_all.py` | Runs the sheets above |

Every dimension, tolerance, font size, line width and layout coordinate is a named constant with a comment
at the top of the module that uses it; derived values are computed.

## Constants other scripts import

| Constant | Module | Value |
|----------|--------|-------|
| `MOUNT_PCD`, `MOUNT_THREAD_D`, `MOUNT_THREAD_PITCH` | `part_02_target_spigot` | 60, 5 (M5), 0.8 |
| `MOUNT_ANGLES_FROM_DOWEL`, `DOWEL_DIRECTION` | `part_02_target_spigot` | 45, 135, 225, 315 degrees from the dowel; `+Ym` |
| `mount_hole_positions()` | `part_02_target_spigot` | (x, y) of the four holes in the target frame |
| `BACK_PLATE_T`, `MOUNT_ENGAGEMENT`, `MOUNT_SCREW_LEN`, `FLANGE_D`, `FLANGE_T` | `part_02_target_spigot` | 8, 6, 10, 80, 8 |
| `STANDOFF_D`, `STUD_THREAD_D`, `STUD_PITCH`, `STANDOFF_LENGTHS` | `part_03_standoffs` | 10, 5, 0.8, (15, 60) |
| `STUD_FRONT_LENGTH_MM`, `STUD_BACK_LENGTH_MM` (`STUD_LEN` = back) | `part_03_standoffs` | 5.5, 8 |
| `stud_recess(plate_t, stud_len)` | `part_03_standoffs` | how far a stud end lies below the far face of a through-tapped plate |
| `KEYWAY_DEPTH_MM` | `part_01_target_adapter` | 4.5 |

`part_02_target_spigot.py` and `part_03_standoffs.py` compare these with `spigot_pattern.py` (used by the
plate drawings) and stop with an error if the two disagree.

## Points the shop must confirm (also on the sheets)

PT-04 to PT-07 (target plates):

- G (15 or 60 mm) is the depth step between the front face of the front part and the front face of the back plate (the specification's meaning). Standoff body = G minus the front part's thickness: 9 / 54 for 6 mm parts, 5 / 50 for the 10 mm T3a square (read from PT-03's `STANDOFF_BODY_LENGTHS_MM`). On T5 the 8 mm bracket then leaves only 1 mm to the back plate at G = 15.
- The brief's 6 mm blind M5 holes cannot be made in a 6 mm plate: front plates (PT-05, PT-07) have M5 through-tapped holes with the 5.5 mm front stud ending 0.5 mm below the front face (the plate is bead-blasted before assembly, so the open holes stay in the blasted face); the T3a square is 10 mm thick with blind M5 holes, thread 6 deep, drilled 8.0 deep to the tip of a 118 degree point (full diameter to 6.74 mm, 0.74 mm below the thread; floor under the tip 2.0 mm). The standoff square is 64 mm (holes at +/-32), so the thread edge clears the 80 mm flange rim by 2.75 mm.
- PT-06: disk post bores are 2 H7, depth = disk thickness - 0.5 mm (1.5 / 2.5 / 3.5), so every disk post is G + 5.5 long (20.5 at G = 15, 65.5 at G = 60) with a free length of G minus the disk thickness (11 / 12 / 13 and 56 / 57 / 58). The post-only post has free length G - 2 (13 / 58) and is 19 / 64 long. Posts are not blasted (dull, as drawn).
- PT-06: the post feature in the code has a diameter of 2.034 mm; the post is made from 2.0 mm rod.
- PT-07: four standoffs (x = +156 and x = -120, y = +/-87), not six; the bracket carries three M5 x 8 DIN 7984 screws (M5 x 10 would protrude 0.5 mm), and the front face of the bracket extension is bead-blasted because the sensor can see it. The engraved orientation arrow is on the bar's back face (x = -148), not on the extension (too narrow).
- PT-07: nearest hole (standoff or bracket screw) to a blank site is 7.1 mm; the front plate is 6 mm thick, the bracket screws have only 4.5 mm of thread engagement.

- PT-01: the Ø11 counterbores of the flange screws reach r = 19.5, 0.5 inside the Ø40 bore (four notches,
  6.5 deep); the M6 heads (Ø10) just touch the bore radius.
- PT-01 keyway 4.5 deep leaves 0.5 radial clearance over the 4 mm PT-02 dowel (bottom 44.5 across the bore).
- PT-01 dowel hole is 8 deep so that its drill point stays 1.2 from the Ø8 cross hole.
- PT-02 screw: M5 x 10 = flange 8 - counterbore 4 + 6 engagement; the back plate is tapped through.
- PT-03 studs: front M5 x 5.5 (ends 0.5 below the face of a 6 mm through-tapped plate; also seats in the 10 mm raised
  square), back M5 x 8 (flush in the 8 mm plate); the groove 0.5 x 0.3 on the body marks the front end.
