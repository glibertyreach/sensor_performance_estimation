# Review decisions log — VSX3000 characterization specification

Source of truth: the live Claude Docs document. Decisions are logged here as they are made and executed
only when the person closes the chunk. "Agreed" items are the reviewer's proposals the person accepted.

## C01 — Section 1, Purpose and scope (opened 2026-10-05)

Person's comments:
- C01-P1: the phrase "rise distance" is unusual and needs a definition (first use is in the measurand
  table, B-HV row; the definition belongs where the term first appears, with the 10–90 percent rule
  of Section 11.1 Step 6 stated in one sentence).

Agreed reviewer proposals (executed 2026-10-05, document rev 45 + flow-diagram republish):
- C01-R1: counts: "five properties" in the opening sentence; figure caption "6 capture series, 6 analyses"
  with registration counted; figure label text to match.
- C01-R2: dependency claim restated: A supplies sigma_tot to E and to the warm-up and flatness gates;
  B-Z and D set their thresholds from their own null data (A-to-A visits, blank sites); figure arrow
  "σ, τ" relabeled.
- C01-R3: baseline side: "the right camera is taken to lie at +H; Step 4.5 confirms the side" (and the
  side check added to Step 4.5 when C05 is reviewed).
- C01-R4: forward-model mapping sentence gains E's boundary terms (W_fab, W_drop, π_near).
- C01-R5: B-HV outputs "and gap"; out-of-scope sentence "A–E".

Status: closed 2026-10-05. Executed in the live document: C01-P1 (rise-distance definition in the B-HV
measurand cell), C01-R1 ("five properties"; caption "6 capture series, 6 analyses"), C01-R2 (dependency
sentence; flow-diagram label "σ, τ" → "q"; new A → E connector labeled "σ_tot"), C01-R3 (right camera at +H,
Step 4.5 checks direction and side; the Step 4.5 side check itself is deferred to C05), C01-R4 (W_fab, W_drop,
π_near in the mapping sentence), C01-R5 ("polarity, and gap"; "A–E").
Gate: ALL CHECKS PASSED against baseline 20261005_124738; scope diff = Section 1 only.
Deferred follow-up: C05 — add the baseline-side check to Step 4.5 (from C01-R3).

## C02 — Section 2, Parameters (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until the person closes the chunk):
- C02-R1 (serious): the opening claim "every arbitrary constant ... is a named parameter listed here" is
  false; literals remain in Sections 4–13 (750 mm reference Z; warm-up 1 min / 10 frames; settle check
  100 frames / 10 %; sentinel 30 frames; tilt 50 frames; C field sub-series 10 poses; staircase 3 quanta /
  10 frames; pose-log 0.01 mm; continuous-angle 10 mm / 20 poses; fine ladder 2^(1/4); lapse bound 0.05;
  quantization patch 20 × 20 px; autocorrelation 1/e; ESF 10–90 %; flatness fraction 0.25; budget 10 fps /
  3 s). Proposal: add rows now with the code's names; replace each literal in its own chunk.
- C02-R2 (serious): FRAMES_PER_NOISE_STATION says it covers tilt poses (100) but Section 5 Step 5 and the
  code use 50 frames per tilt pose. Proposal: new row FRAMES_PER_TILT_POSE = 50; Meaning of the 100 row
  reads "Frames per Z and field pose".
- C02-R3: DETECTION_FALSE_ALARM_TARGET Meaning names the blank-site distribution; in B-Z the null is the
  A→A visit-difference distribution (Section 11 Step 3). Proposal: "from the null distribution: blank sites
  in D, A→A visit differences in B-Z".
- C02-R4: PHASE_JITTER_SPAN_PX asserts it exceeds the dot pitch and the correlation length, both unknown
  before A is analyzed (Section 10 Step 7 checks it after B, C, D are already captured). Proposal: mark †,
  word it as a requirement, and add a quick-look autocorrelation check between A and B capture (Section 5,
  to be executed in C06).
- C02-R5: POST_DIAMETER_FRACTION_OF_D0 depends on a pilot D_0 that exists only after the posts are built.
  Proposal: Meaning states the provisional source (Tier-A model estimate before fabrication; the D pilot
  and post-only sites confirm).
- C02-R6: ROBOT_REPEATABILITY_MM Meaning: "a rung smaller than this is ... flagged" is vacuous now that the
  smallest rung is 0.1 mm; and repeatability is not the read-back accuracy. Proposal: reword to the truth
  ratio reported with δ_50.
- C02-R7: PLATE_FLATNESS_MM presumes σ_tot ≥ 0.2 mm at Z_MIN (unverified). Proposal: mark †; name the 0.25.
- C02-R8 (style): first person "I do not have" → "not available at the time of writing".
- C02-R9 (low): DEPTH_LSB_MM Used-in "A" → "A, B-Z"; "Used in" column mixes letters and section numbers;
  TILT_ANGLES_DEG includes 0°, which repeats the main-series pose (state it as a deliberate repeat or drop it).
- C02-N1 (note, no action): the relations block exports as raw LaTeX in the docx; verify the live rendering.

Person's comments: none; closed 2026-10-05 with "closed. next" (treated as acceptance of R1–R9).
Executed (document rev 54): R1 added 30 rows using the code's names (Z_REFERENCE_MM, WARMUP_CHECK_*, SENTINEL_FRAMES,
SETTLE_CHECK_FRAMES, SETTLE_SIGMA_EXCESS_FRACTION, FRAMES_PER_TILT_POSE, POSE_LOG_RESOLUTION_MM, FRAME_CHECK_PX,
FRAME_RATE_HZ, MOVE_AND_SETTLE_TIME_S, FIELD_SUBSERIES_POSES_AREA, Z_STAIRCASE_QUANTA, Z_STAIRCASE_FRAMES,
ZSTEP_PATCH_SIZES_PX, DETECTION_FINE_LADDER_RATIO, DETECTION_LAPSE_RATE_MAX, INDEPENDENCE_SIGMA_MULTIPLE,
CONTINUOUS_ANGLE_STEP_MM, CONTINUOUS_ANGLE_POSES_PER_STEP, a new group 'Noise, edge, and area analysis (A, B, C)' with
NOISE_CLOSURE_TOLERANCE, QUANTIZATION_PATCH_PX, AUTOCORRELATION_THRESHOLD, LEGACY_BOX_HALF_PX, LEGACY_METRIC_DEPTHS_MM,
ESF_RISE_LOW/HIGH, ESF_HALF_HEIGHT, ESF_AGREEMENT_BINS, ESF_LINEARITY_TOLERANCE_H, FRONT_READ_HEIGHT_THRESHOLD, and
PLATE_FLATNESS_SIGMA_FRACTION); R2–R8 cell edits as proposed; R9 DEPTH_LSB_MM 'A, B-Z' and the 0° tilt note; the
'Used in' column left mixed (letters and section numbers).
Gate: ALL CHECKS PASSED against baseline 20261005_130223; scope diff = Section 2 only.
Deferred follow-ups: replace each literal with its parameter name in its own chunk (C05 §4: 750 mm, 1 min, 10 frames,
100 frames, 10 %, 0.5 px; C06 §5: 750 mm, 30 sentinel frames, 50 tilt frames; C07 §6: 0.01 mm, 3 quanta, 10 frames;
C08 §7: 750 mm, 10 poses; C09 §8: 750 mm, 2^(1/4), 10 mm, 20 poses; C10 §9: 10 fps, 3 s; C11 §10: 20 × 20 px, 1/e,
0.2 closure, legacy 700/1000 mm; C12 §11: 10–90 %, 0.5, 1 bin, 0.1 h, patch sizes; C13 §12: h ≥ 0.5; C14 §13: 0.05
lapse, ±2/√n). C06 §5: add the quick-look autocorrelation check between A and B capture (from R4).

## C03 — Sections 3.1 Equipment and 3.2 Targets (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until the person closes the chunk):
- C03-R1 (serious, truth chain): 3.1 does not require what B-Z now rests on. The robot row asks only for ISO 9283
  repeatability; the read-back pose is the step truth, so 3.1 must require that the controller reports the actual
  (encoder-derived) pose, at POSE_LOG_RESOLUTION_MM, with a stated pose-reporting accuracy, and that the pose is
  time-stamped against the frames. Proposal: reword the robot row and the capture-PC row to name
  ROBOT_REPEATABILITY_MM and POSE_LOG_RESOLUTION_MM, and add "actual, not commanded" as a requirement here (the
  Section 15 checklist item then becomes a verified requirement rather than an open question).
- C03-R2 (serious, consistency): "Targets can be swapped without re-registering" holds only if each target's feature
  geometry relative to the dowel pins is known. The as-built record of 3.3 measures feature geometry on the plate but
  says nothing about the plate-to-adapter datum. Proposal: state in 3.2 that every target carries the same dowel
  datum and that the as-built record includes the datum-to-feature offsets; Step 4.8's once-per-mount plane check
  then verifies Z and tilt, and the datum record covers H and V.
- C03-R3 (literals): 3.1 and 3.2 carry four unnamed constants: adapter re-mount repeatability 0.02 mm, temperature
  logging 1 sample/min, T1 pattern geometry tolerance 0.02 mm, T1 size 400 × 300 mm. Proposal: name them
  (ADAPTER_REMOUNT_REPEATABILITY_MM, TEMPERATURE_LOG_INTERVAL_MIN, PATTERN_GEOMETRY_TOLERANCE_MM,
  REGISTRATION_PLATE_SIZE_MM) in Section 2 (scope: Section 2 touched again, declared as intended) and cite them here.
- C03-R4 (omission): T5 construction says the back plate sits at G, but Section 7 Step 4 removes it for the
  open-background variant. Proposal: "back plate at G, removable for the open-background variant of Section 7".
- C03-R5 (omission): plate extents are unspecified for T3a/T3b back and front plates and for the T4/T5 arrays. The
  edge analyses need the back plate to extend past the square by at least the boundary band plus the shadow at
  GAP_LARGE_MM at the worst ray angle; the arrays need room for 16 diameters, blank sites per window size, and
  post-only sites at FEATURE_ISOLATION_PX. Proposal: one sentence per target family giving the rule (not a number),
  with the plan tool's layout (make_feature_array) as the source of the as-built drawing.
- C03-R6 (claim check): "about 0.3 mm to 60 mm in about 16 steps" at f_x ≈ 500 px reconciles (p(500) = 1.0 mm,
  p(1000) = 2.0 mm, ln 200 / ln √2 = 15.3). No change; the worked example should say it is superseded once
  SENSOR_FX_PX is known.
- C03-R7 (low): the 3.1 robot row should also require the approach-from-below move (Z_STEP_APPROACH_OVERSHOOT_MM) to
  be programmable, and the enclosure row should name the IR meter's logging interval (same as the temperature
  loggers). Surface-finish paragraph: "mid-range IR reflectance" is unquantified; propose recording the measured
  reflectance as a required field in targets_asbuilt.csv rather than fixing a number.
- C03-Q1 (person's pending decision, not executed): merging T1 and T2 into one plate registered by plane
  correspondence (asked earlier in chat). If adopted, the T1 row changes to "plane-registration plate = T2" and
  the ChArUco pattern is dropped; the registration method in Section 4 changes with it.

Person's comments (2026-10-05):
- C03-P1: remove the "Enclosure or blackout curtains, IR light meter" row; the laboratory is enclosed and its lighting
  is constant. Propagation: the ambient-IR manifest column (Section 9, C10) and any ambient-IR mention in Sections 4
  and 15 go with it; the R7 note on the IR meter's logging interval is void.
- C03-P2: the person does not like the pattern on the registration plate and asks whether it can be eliminated
  (same question as Q1: plane-correspondence registration with the T2 plate). Answer given in chat with the
  consequences (rotation and camera position stay observable; the camera-Z offset is conditioned by the tilt
  range; the in-plane position of a target's features relative to the flange is not observable from planes and must
  come from the dowel datum plus as-built metrology, or from an IR-image localization once per mount).
- C03-P3: the person questions the need for a large number of disks and holes and asks how many are proposed.
  Answer given in chat: 17 rungs (0.22 to 55.8 mm at the indicative f_x = 688 px); 17 disks + 17 blank sites +
  4 post-only sites over T4-S/T4-L, 17 holes + 17 blank sites over T5-S/T5-L, 74 sites in all; reduction options
  offered (12 rungs keeping √2 only over 0.5–8 mm, or 8 rungs at ratio 2).
Awaiting the person's decisions on P2 and P3 before closing the chunk.
- C03-P3 decision (2026-10-05): the person rejects the 17-rung ladder. Directive: radically reduce the number of
  disks and cutouts and use robot motion in Z to vary the subtended angle. Reviewer proposal put to the person:
  6 disks and 6 holes (a 5-rung ratio-2 ladder 0.44–6.98 mm whose D_px ranges tile 0.3–9.6 px over Z_MIN..Z_MAX,
  plus one 27.9 mm asymptote feature), one disk plate and one cutout plate, blank and post-only sites as named
  parameters, C and D sampled at the 11 A stations in Z, θ-scaling tested where adjacent disks overlap.
  Propagation when adopted: Section 2 rows (DIAMETER_*), 3.2 T4/T5 rows and ladder paragraph, Sections 7, 8, 9
  (budget), 12, 13, the code (targets, plan, analyses C and D, simulator, tests) and the technician procedure.
- C03-P3 (2026-10-05, second directive): for the disk and cutout experiments the robot distance may vary from
  400 mm to 1600 mm (a factor 4, two octaves of subtended angle). Reviewer proposal put to the person: 4 disks and
  4 holes at ratio 2√2 (0.70, 1.97, 5.58, 15.8 mm at f_x = 688 px), each spanning two octaves of pixel size with a
  half-octave overlap at every junction for the θ-scaling test; 9 Z stations log-spaced at 2^(1/4) from 400 to
  1600 mm; feature isolation set at the far station; noise at each station taken from the plate's own blank regions
  (A stays at 500–1000 mm). Alternative: 3 disks at ratio 4 without overlap.
- C03-P3 (2026-10-05, third directive, person's domain knowledge): the 0.70 and 1.97 mm features would never be
  detected. VSX3000 model: about 300,000 depth voxels and about 30,000 projected laser pencils, 10 voxels per
  pencil; the path-correlation algorithm needs several pencil detections, at least 4, so a patch of at least 40
  voxels, a minimum detectable diameter of at least 6 px (2·sqrt(40/π) = 7.1 px). The person calls this very
  conservative and asks the reviewer to check available sources and consider still larger patch sizes. The person
  also allows the 400–1600 mm distance variation for the rest of the tests if necessary.
