# Procedure-docs pass: decision log (opened 2026-10-07)

Method: the procedure-docs skill (scope of work section 1a to 1f, shop drawings, two decks), applied to the
VSX3000 performance-testing procedure. Decisions settled by the characterization specification are marked
"settled"; the rest are asked one or two at a time and the answers logged here.

| # | Topic | Status | Answer / source |
|---|---|---|---|
| 1 | Targets | settled | T2 noise plate 400 x 400 mm; T3a raised square 160 mm; T3b square window 160 mm; T4 disk plate (7.0, 19.7, 55.8 mm disks, 3 blank sites, 1 post site); T5 cutout plate (same ladder, 3 blank sites); gaps 15 and 60 mm by spacers (specification Sections 3.2 and 3.3) |
| 2 | Size tolerance | settled | fabrication tolerances left to the fabricator; every feature measured as built (diameter, land, bevel, position, datum offset) with uncertainties into targets_asbuilt.csv |
| 3 | Form | settled | plates flat to 0.05 mm; land 0.2 mm or less; bevel from the back, 45 degrees passes at the indicative geometry |
| 4 | How size is measured | answered (Q2) | fabricator's inspection report; in-house only plate flatness (run-out gauge), width and length; sphere diameter by micrometer if a sphere is ever used |
| 5 | Finish | settled | one finish on every front and back surface, bead-blasted aluminum or matte coating, mid-range IR reflectance, recorded |
| 6 | Mounting | settled | dowel-pinned quick-change adapter, re-mount repeatability 0.02 mm, every target on the same dowel datum; posts 2 mm or thinner behind the disks and the raised square |
| 7 | Robot flange | answered (Q1) | robot TBD; drawn to ISO 9409-1-50-4-M6 with the confirm note; documents agnostic of the model |
| 8 | Robot accuracy | settled | repeatability 0.05 mm (ISO 9283), read-back pose at 0.01 mm time-stamped against the frames; documented specification, not a calibration |
| 9 | Independent check | settled | no ball bar; registration residual and plane fits are the robot-independent checks; the bar is never mentioned |
| 10 | Capture trigger | settled | the VSX3000 SDK with the LRVisionLibs MatCloud reader and the existing robot interface |
| 11 | Sensor processing | settled | production configuration recorded in sensor_config.json; optional filters-off repeat |
| 12 | Robot program | settled | written with Claude Code once the robot model and controller are settled |
| 13 | Items not needed | settled | lighting control (enclosed laboratory, constant lighting); file transfer |
| 14 | Sensor mount | ask (Q3) | the specification requires a rigid stand separate from the robot; whether an existing drawing exists is unknown |
| 15 | Drift-run fixed stand | ask (Q3) | a stand for T2 at 800 mm, robot idle; drawn or bought is open |
| 16 | Decks | settled | two: procurement and mechanical build; test procedure and robot program |
| 17 | Status slides | settled | none |
| 18 | Notes PDF | ask at delivery | offered once when the decks exist |

Questions asked, in order, and the answers:

- Q1 (robot and flange), Neil 2026-10-07: the robot model is unknown (TBD); the specification stays agnostic of the robot model. Consequence: the adapter is drawn to ISO 9409-1-50-4-M6 with the boxed confirm note, as in the related projects; no robot is named anywhere.
- Q2 (metrology), Neil 2026-10-07: no as-built measurements of the targets in-house beyond a sphere's diameter with a micrometer, a plate's flatness with a run-out gauge, and a plate's width and length. Resolution proposed and agreed: the fabricator delivers an inspection report (feature diameter, land, bevel, position from the dowel datum) that fills targets_asbuilt.csv; a value not reported falls back to the nominal with the drawing tolerance as its uncertainty, so the drawings carry tolerances; in-house the technician records each plate's flatness, width and length. This changes the specification's Section 3.3 (as-built record) and is to be executed there as well.
- Related projects named by Neil for consistency and reuse: "Binocular depth sensor calibration" = glibertyreach/depth_calibration_from_spherical_target (stage-1 decks, drawings SC1-01 to SC1-06, cost pattern); "Sensor registration via plane correspondence" = glibertyreach/plane_plane_registration (procedure with the same Section 1 structure, costs.py, decks, the consistency-pass skill). Equipment and procedures are reused from them wherever they fit.
- Q3 (target mounting) and Q4 (drift-run stand), presented 2026-10-07 and 2026-10-08; Neil asked why the noise plate
  is 400 mm and how the current (stage-1) plate could serve. Reviewer answer: the size was a design choice (far-station
  ROI of about 170 px, 40 rows per ramp quantum, registration leverage); the stage-1 board (200 x 150 mm on SC1-05)
  serves with the ramp spanning 2 quanta instead of 4, a 5,500-pixel far-station ROI, tilts feasible from the second
  station, and the same registration hardware as the registration project.
- Neil's decision (2026-10-08): adopt; the 400 mm noise plate is eliminated. T2 = the stage-1 board on its SC1-05
  adapter; NOISE plate size 200 x 150 mm; RAMP_QUANTA = 2. Q4 is thereby answered: the drift run uses the board on any
  rigid laboratory stand, as the registration project does; no stand drawing.
- Q3 re-presented for the feature targets (T3a, T3b, T4, T5) only; pending.
- Neil's question (2026-10-08): what is lost if spheres are eliminated and only planes are used, after trying to
  measure the lost quantities with planes. Reviewer answer given in chat (curvature term of the depth bias becomes a
  prediction from the measured line spread function; grazing incidence beyond the tilt range unmeasured; lateral
  position, registration translation, angular reflectance to the tilt limit and the tool frame are recoverable with
  edged plates and tilts).
- Q3 answered (Neil, 2026-10-08): the spigot-and-cross-pin design for the four feature targets (T3a, T3b, T4, T5).
- Sphere decision (Neil, 2026-10-08): one sphere, the larger (stage-1 sphere B, 152.4 mm); the curvature sweep runs
  in the calibration procedure, not here. Handoff written: docs/design/handoff_sphere_decision_2026-10-08.md.

