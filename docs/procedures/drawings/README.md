# VSX3000 performance testing: shop drawings

Dimensioned shop drawings (millimeters, third-angle projection, general tolerances ISO 2768-mK, date
2026-10-08) for the fixtures of the performance test. Each sheet is a 3300 x 2100 px PNG (200 dpi,
landscape) with its views at one stated scale, a notes block and a title block.

| Drawing | Part | Qty | Material | Scale | File |
|---------|------|-----|----------|-------|------|
| PT-01 | Target adapter (120 x 120 x 30 plate; ISO 9409-1-50-4-M6 flange side; Ø40 H7 bore, keyway, cross hole) | 1 | Aluminum 6061-T6 | 1:1 | `PT-01_target_adapter.png` |
| PT-02 | Target spigot (Ø40 h6 body, Ø80 flange, dowel, cross hole; detail M, mating pattern on each target back plate) | 4 | 303 or 17-4 PH stainless | 1:1 | `PT-02_target_spigot.png` |
| PT-03 | Standoff set (Ø10 body; M5 x 8 back stud, M5 x 5.5 front stud, front end marked by a groove; L = 15 and L = 60) | 28 (14 + 14) | Aluminum 6061-T6 | 2:1 | `PT-03_standoff_set.png` |

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
| `part_03_standoffs.py` | PT-03: end view and elevations of both lengths |
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

- PT-01: the Ø11 counterbores of the flange screws reach r = 19.5, 0.5 inside the Ø40 bore (four notches,
  6.5 deep); the M6 heads (Ø10) just touch the bore radius.
- PT-01 keyway 4.5 deep leaves 0.5 radial clearance over the 4 mm PT-02 dowel (bottom 44.5 across the bore).
- PT-01 dowel hole is 8 deep so that its drill point stays 1.2 from the Ø8 cross hole.
- PT-02 screw: M5 x 10 = flange 8 - counterbore 4 + 6 engagement; the back plate is tapped through.
- PT-03 studs: front M5 x 5.5 (ends 0.5 below the face of a 6 mm through-tapped plate; also seats in the 10 mm raised
  square), back M5 x 8 (flush in the 8 mm plate); the groove 0.5 x 0.3 on the body marks the front end.
