# Consistency pass: performance_test_procedure.md against 2 derived documents

Governing document: `docs/procedures/performance_test_procedure.md` (built from the template on 2026-10-08, baseline 2026-10-08_07).
Derived documents: `docs/presentations/vsx3000_procurement_build.pptx` (17 slides) and `docs/presentations/vsx3000_test_procedure.pptx` (20 slides).
The decks are built from `perf_build_deck_content.json` and `perf_procedure_deck_content.json`; the content check confirms every slide string and note equals its JSON source, so the JSON key paths below locate the statements in the decks.

## Conflicts

| # | Governing statement (id, quoted, location) | Conflicting statement (quoted, location) | Kind | Proposed fix |
|---|---|---|---|---|
| 1 | G1: "each target carries a spigot (PT-02) bolted to the back of its back plate" (section 1b, paragraph after Table 1) and "Spigot PT-02 on the back of the back plate: four M5 tapped holes on a 60 mm pitch circle ... same, same, same" (section 1d, Table 3, Mounting interface row, T5 column) | "T5's spigot sits on an edge bracket so the back plate can be removed" (build deck slide 7, table row Mounting, T5 column; `slides[6].table.rows[6][2]`) and "T5, cutout plate with its edge bracket" (slide 4 table; `slides[3].table.rows[6][0]`) | changes the mounting of T5 | The procedure changes: Appendix F already carries the edge bracket, so section 1b and the T5 cell of Table 3 should say that T5's spigot sits on the edge bracket PT-07.3 at the plate's left edge because the back plate is removable. The decks agree with the drawings. |
| 2 | G2: "the worst field position at Z_MIN gives about 32 degrees. With the margin that is 42 degrees, so a standard 45 degree bevel (90 degree included countersink) would pass" (section 2, chamfered boundaries, worked value) | "The bevel angle must exceed the worst-case ray angle plus a 10 degree margin; at the indicative geometry that is 39 degrees, so the standard 45 degree bevel passes" (build deck slide 3, notes; `slides[2].notes`) | changes a number | The deck notes change to 42 degrees, or the procedure's worked value is brought to the specification's current 29 degrees plus 10 (39 degrees, specification Section 3.3); the two source documents disagree with each other here and one of them should be corrected first. |
| 3 | G3: "Measure the real gap with the depth rod of the calipers and record it" (section 2, Two planes); the as-built gap is the step the sensor sees (section 1b) | "Standoffs measured and grouped; the measured lengths recorded as the gaps" (build deck slide 17, checklist; `slides[16].checklist[4]`) | changes what is recorded (a standoff body is G minus the front part's thickness, not G) | The deck changes: "Standoffs measured and grouped; the gap recorded is the measured body length plus the front part's thickness, checked with the depth rod". |
| 4 | G4: "hidden behind the raised square of T3a (on a 64 mm square)" (section 1b) and "the 64 mm square behind the raised square" (Appendix F) | "The hidden standoffs of T3a sit on a 60 mm square" (build deck slide 14, notes; `slides[13].notes`) | keeps a value the governing document has since changed | The deck notes change to 64 mm. |
| 5 | G5: "Every plate flat to 0.05 mm over its front face, checked by the fabricator before finishing and in-house with the run-out fixture after mounting" (section 1d, Table 3, Flatness row) | "0.05 mm over each front face, checked in-house with the run-out fixture after mounting" (build deck slide 7, table row Flatness; `slides[6].table.rows[4][1]`); the slide's notes do not carry the fabricator's check | drops a condition (the fabricator's flatness check before finishing) | The deck changes: add "checked by the fabricator before finishing and" to the row, or carry it in the notes. |

## Gaps in the governing document

| # | Derived statement (quoted, location) | What the governing document should say |
|---|---|---|
| 1 | "Each spigot drops into the adapter, its dowel in the keyway, and the ball-lock pin passes; no play" (build deck slide 17, checklist; `slides[16].checklist[1]`) | Section 1f or 2 should list a fit check of every spigot in the adapter before the first session (no play, pin passes), since the mount check of Step 4.8 measures only the mounted target's pose. |
| 2 | "Screw torques recorded; witness lines marked across the adapter and the flange" (build deck slide 17, checklist; `slides[16].checklist[7]`) and "a witness line across the adapter and the flange shows whether it has moved" (slide 11, notes) | Section 1f (Robot) should ask for the witness line across the adapter and the robot flange after tightening, as drawing PT-01 note 10 does; the procedure mentions only the recorded torques (section 1c). |
| 3 | "Aluminum, 10 mm body ... plus or minus 0.05 mm" (build deck slide 13, points; `slides[12].points[0]`) | Table 3 of section 1d states no material or tolerance for the standoffs; it could point to drawing PT-03 for both, as it does for the spigot. |
| 4 | "posts as drawn, dull; do not blast" (drawing PT-06, and the deck's slide 3 card "One finish ... every face the sensor sees") | Section 1d, Surface row, says "same, disks and posts included" for T4, while drawing PT-06 says the posts are not blasted; the procedure should say which. |

## Not attachable (for a skim, not defects)

- Build deck slide 1 notes: the robot model is not yet chosen (restates section 1, not a requirement).
- Build deck slide 6 notes: the cost figures come from `costs.py` (tooling, not procedure content).
- Build deck slide 11 to 15 points and notes: dimensions read from the drawings (31.5 mm g6 spigot, M6 x 30 screws, 82 mm bar, counterbores, groove, 118 degree drill point); Appendix F embeds the drawings but its text does not restate these numbers.
- Build deck slide 9 table: supplier names and price ranges beyond Appendix E's list (iGaging, Jergens) are additions, not conflicts; Appendix E names Jergens and iGaging too.
- Procedure deck slide 2 message and slide 12 message: rationale ("If the manifest is wrong, the results are wrong"; "the robot program can be written with Claude Code") restates the introduction and section 1f.
- Procedure deck slide 13 stats "3 s" and "10 fps": section 13's budget assumptions, not requirements on the technician.
- Procedure deck slide 20, Software: tool names in `sensorperf.cli` (Appendix B).

## Coverage

Read in full: the governing procedure's extracted text (1,290 lines: sections 1 to 15, appendices A to F including the drawing table and the points for the shop); both deck content files (every slide's title, table, cards, points, steps, stats, captions, checklist and notes; 333 and 370 lines). The decks' slide text and notes equal the content files by the content check (`check_perf_content.py`, 0 problems); the pptx files were extracted after the final build to confirm (see the note at the end). Generated fields: the cost tables and the cost paragraph of section 1 and the cost slide's figures come from `costs.py`, and the plate sizes, masses, disk space and budget numbers of the procedure from `build.py`; those were checked as well and agree. The drawings (PNG) are images and were not text-extracted; their notes were read when the sheets were reviewed, and gap 4 comes from that review. Not covered: the characterization specification (Claude Docs), which is not among the three documents named; conflict 2 notes one place where it and the procedure disagree.
