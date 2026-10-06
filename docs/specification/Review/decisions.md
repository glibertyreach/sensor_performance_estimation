# Review decisions log — VSX3000 characterization specification

Source of truth: the live Claude Docs document. Decisions are logged here as they are made and executed
only when Neil closes the chunk. "Agreed" items are the reviewer's proposals Neil accepted.

## C01 — Section 1, Purpose and scope (opened 2026-10-05)

Neil's comments:
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

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
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

Neil's comments: none; closed 2026-10-05 with "closed. next" (treated as acceptance of R1–R9).
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

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
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
- C03-Q1 (Neil's pending decision, not executed): merging T1 and T2 into one plate registered by plane
  correspondence (asked earlier in chat). If adopted, the T1 row changes to "plane-registration plate = T2" and
  the ChArUco pattern is dropped; the registration method in Section 4 changes with it.

Neil's comments (2026-10-05):
- C03-P1: remove the "Enclosure or blackout curtains, IR light meter" row; the laboratory is enclosed and its lighting
  is constant. Propagation: the ambient-IR manifest column (Section 9, C10) and any ambient-IR mention in Sections 4
  and 15 go with it; the R7 note on the IR meter's logging interval is void.
- C03-P2: Neil does not like the pattern on the registration plate and asks whether it can be eliminated
  (same question as Q1: plane-correspondence registration with the T2 plate). Answer given in chat with the
  consequences (rotation and camera position stay observable; the camera-Z offset is conditioned by the tilt
  range; the in-plane position of a target's features relative to the flange is not observable from planes and must
  come from the dowel datum plus as-built metrology, or from an IR-image localization once per mount).
- C03-P3: Neil questions the need for a large number of disks and holes and asks how many are proposed.
  Answer given in chat: 17 rungs (0.22 to 55.8 mm at the indicative f_x = 688 px); 17 disks + 17 blank sites +
  4 post-only sites over T4-S/T4-L, 17 holes + 17 blank sites over T5-S/T5-L, 74 sites in all; reduction options
  offered (12 rungs keeping √2 only over 0.5–8 mm, or 8 rungs at ratio 2).
Awaiting Neil's decisions on P2 and P3 before closing the chunk.
- C03-P3 decision (2026-10-05): Neil rejects the 17-rung ladder. Directive: radically reduce the number of
  disks and cutouts and use robot motion in Z to vary the subtended angle. Reviewer proposal put to Neil:
  6 disks and 6 holes (a 5-rung ratio-2 ladder 0.44–6.98 mm whose D_px ranges tile 0.3–9.6 px over Z_MIN..Z_MAX,
  plus one 27.9 mm asymptote feature), one disk plate and one cutout plate, blank and post-only sites as named
  parameters, C and D sampled at the 11 A stations in Z, θ-scaling tested where adjacent disks overlap.
  Propagation when adopted: Section 2 rows (DIAMETER_*), 3.2 T4/T5 rows and ladder paragraph, Sections 7, 8, 9
  (budget), 12, 13, the code (targets, plan, analyses C and D, simulator, tests) and the technician procedure.
- C03-P3 (2026-10-05, second directive): for the disk and cutout experiments the robot distance may vary from
  400 mm to 1600 mm (a factor 4, two octaves of subtended angle). Reviewer proposal put to Neil: 4 disks and
  4 holes at ratio 2√2 (0.70, 1.97, 5.58, 15.8 mm at f_x = 688 px), each spanning two octaves of pixel size with a
  half-octave overlap at every junction for the θ-scaling test; 9 Z stations log-spaced at 2^(1/4) from 400 to
  1600 mm; feature isolation set at the far station; noise at each station taken from the plate's own blank regions
  (A stays at 500–1000 mm). Alternative: 3 disks at ratio 4 without overlap.
- C03-P3 (2026-10-05, third directive, Neil's domain knowledge): the 0.70 and 1.97 mm features would never be
  detected. VSX3000 model: about 300,000 depth voxels and about 30,000 projected laser pencils, 10 voxels per
  pencil; the path-correlation algorithm needs several pencil detections, at least 4, so a patch of at least 40
  voxels, a minimum detectable diameter of at least 6 px (2·sqrt(40/π) = 7.1 px). Neil calls this very
  conservative and asks the reviewer to check available sources and consider still larger patch sizes. Neil
  also allows the 400–1600 mm distance variation for the rest of the tests if necessary.
- C03 closed 2026-10-05 ("accepted. execute. next."): 3 features per plate at 7.0, 19.7, 55.8 mm (ratio 2√2 from 3 px at
  Z_MAX); one geometric station ladder 400–1600 mm at 2^(1/4) (9 stations) for all series, B-HV at every second and
  B-Z at every fourth station; Z_REFERENCE_MM = 800; registration by plane correspondence on T2 (T1 and its pattern
  removed); in-plane datum from the as-built record and a once-per-mount IR-image check; enclosure/IR-meter row
  removed with the ambient-IR manifest column; robot row requires the actual pose at POSE_LOG_RESOLUTION_MM; new
  named constants ADAPTER_REMOUNT_REPEATABILITY_MM, TEMPERATURE_LOG_INTERVAL_MIN, BLANK_SITES_PER_PLATE,
  POST_SITES_PER_PLATE, DETECTION_ZERO_STATION_COUNT, FEATURE_LADDER_RATIO, FEATURE_MIN_PX_AT_Z_MAX, FEATURE_COUNT,
  Z_STATION_RATIO, Z_SHAPE_STATION_STRIDE, Z_REDUCED_STATION_STRIDE; DIAMETER_*, DETECTION_LEVEL*, FINE_LADDER and
  CONTINUOUS_ANGLE_* rows removed; D pilot reduced to the post check; continuous-angle variant removed (it is the
  design); Sections 12 and 13 pooled in D_px with the scaling test; Section 15 registration rows and limitations
  reworded; open questions on fiducial imaging and pose reporting removed (now requirements). Literal replacements in
  Sections 4, 5, 6, 7, 8 done while those sections were rewritten (C05–C09 deferred items cleared). The Section 9
  budget table is refilled from the plan tool once the code lands. Gate: ALL CHECKS PASSED against baseline
  20261005_131333, scope diff = 1, 2, 3.1, 3.2, 3.3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15 (all intended).
  Figures: flow diagram ("9 Z × 5 fields", "post check"), setup diagram (400–1600 mm volume, enclosed laboratory).

## C04 — Section 3.3 Chamfered (knife-edge) boundaries (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C04-R1 (serious, stale worked value): the paragraph still reasons from "70° FOV, 50 mm baseline" and the old Z_MIN,
  giving 32° + 10° = 42° so that a 45° bevel passes. With Z_MIN = 400 mm the same conservative construction (a
  chamfered target at the off-axis field position at Z_MIN) gives about 38° + 10° = 48° at the indicative geometry
  (f_x ≈ 688 px, 640 × 480, 75 mm baseline), and a 45° bevel fails. But that pose never occurs: chamfered targets
  are off-axis only in the C field sub-series at Z_REFERENCE_MM (800 mm), and at Z_MIN they are centered (B-HV
  edges). Worst real cases: the raised square centered at 400 mm, about 24°; the arrays off-axis at 800 mm, about
  29°; plus the 10° margin, 39°, so 45° passes with margin. Proposal: rewrite the worked value around the poses
  the plan actually uses, name the indicative geometry, keep "recompute once the VSX3000 geometry is known", and
  keep the escape clause (steeper bevel or smaller FIELD_OFFSET_FRACTION).
- C04-R2 (consistency with 3.2): the as-built record lists "plate position" but 3.2 now relies on it for the
  datum-to-feature offsets and the measured reflectance. Proposal: "position relative to the dowel datum" and
  "the measured reflectance at the projector wavelength" named explicitly.
- C04-R3 (code/spec): the plan tool's nominal post diameter is 0.5 mm while the rule gives "2 mm or less"; the
  code constant should be derived from POST_DIAMETER_FRACTION_OF_D0 × the expected D_0 rather than fixed.
  Code follow-up, no document change.
- C04-R4 (low): state that chamfered targets are never tilted (the tilt sub-series uses T2), so the max over poses
  in the bevel formula runs over fronto-parallel poses only; and that the 45° in the figure is the example value,
  not a requirement.
- C04-N1 (note): the bevel formula exports as raw LaTeX in the review PDF; verify the live rendering.

Neil's comments: (pending)
Neil's comments (2026-10-05), closed with "close":
- C04-P1: the LaTeX math in Section 3.3 is not formatted. Finding: the bevel formula is stored in the live document as
  a code block (as are the relations in Section 2, the noise model in Section 10, the psychometric model in Section
  13, and the two boundary-bias formulas in Section 14), so it is unformatted in the live document too, not only in
  the review PDF. To execute: convert each to the document's math block type if the editor has one; otherwise
  typeset each as a figure (SVG) or as Unicode text.
- C04-P2: the figures in the PDF should be numbered. Finding: the live document's figure captions carry no numbers.
  To execute: number the three figures in document order in their captions (Figure 1 flow diagram §1, Figure 2
  chamfer cross-section §3.3, Figure 3 setup side view §4), add the in-text references, and let the renderer and the
  gate's cross-reference check pick them up.
Executed 2026-10-05 (document rev 72): R1 worked value rewritten around the poses the plan uses (24° centered at
Z_MIN, 29° off-axis at Z_REFERENCE_MM, 39° with margin, 45° passes; the figure's 45° named as the example); R2 as-built
record names the datum offsets and the measured reflectance; R4 folded into R1; P1 handled in the review pipeline
(the live document already renders its latex blocks; refresh_spec.py now typesets them for the PDF); P2 figure
captions numbered Figure 1–3 with in-text references in Sections 1, 3.3 and 4. R3 is a code follow-up (post
diameter derived from the rule).

## C05 — Section 4 Setup, warm-up, and registration (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C05-R1 (serious): the settle and vibration check runs at Z_MAX, where σ_t is largest (it grows as Z²: 16× from
  400 to 1600 mm) while a robot vibration has the same millimeter amplitude at every Z, so the 10 % excess test is
  least sensitive exactly where it is run. Proposal: run it at Z_MIN (optionally also at Z_MAX).
- C05-R2 (serious): the mount check applies FRAME_CHECK_PX (a pixel tolerance) to Z and tilt as well as to H and V.
  Proposal: Z within REGISTRATION_RESIDUAL_ACCEPT_MM, tilt within a new MOUNT_TILT_TOLERANCE_DEG (suggested 0.05°,
  the tilt that moves a plate edge 200 mm from center by about 0.17 mm; value uncertain), H and V within
  FRAME_CHECK_PX; add the row to Section 2.
- C05-R3: the registration pose set must make the plate normals span three dimensions: tilts about H and about V,
  of both signs, at several Z; and the solve must report the standard error of the camera's Z offset from the fit
  covariance (the quantity Section 15 estimates at about 0.1 mm) as an acceptance output beside the RMS residual.
  Note that larger tilts are bounded by the plate staying in the field of view.
- C05-R4: the warm-up gate compares drift with σ_t before A has measured it. Proposal: state that σ_t here is the
  temporal standard deviation of the warm-up frames themselves.
- C05-R5 (style): step references mix "Step 8" and "Step 4.7". Proposal: "Step 4.N" throughout.
- C05-R6 (low): the filters-off repeat of A and B is outside the Section 9 budget; say so. Record the SDK and
  firmware versions in sensor_config.json.
- C05-N1 (for C06): the plan tool reports that the 400 × 400 mm noise plate does not fill the field at the near
  stations off-axis; Section 5's "check that the plate covers the region of interest" needs a rule.
- Pipeline: numbered steps now render as numbered lists (nested sub-steps inside their step); fixed in
  render_subsection.py during this chunk.

Neil's comments: none; closed 2026-10-05 with "close" (treated as acceptance of R1–R6; N1 carried to C06).
Executed (document rev 77): R1 settle check at Z_MIN with the reason stated; R2 mount check with Z within
REGISTRATION_RESIDUAL_ACCEPT_MM, tilt within the new MOUNT_TILT_TOLERANCE_DEG (0.05 †, row added to Section 2),
H and V within FRAME_CHECK_PX; R3 registration poses with tilts about both axes and both signs, plate kept in the
field, and the standard error of the camera Z offset reported and saved beside the residual; R4 σ_t of the warm-up
gate defined from the warm-up frames; R5 "Step 4.8" in Sections 3.2 and 4; R6 filters-off repeat noted as outside
the budget, SDK and firmware versions recorded. Code: mount_tilt_tolerance_deg added to the parameters; check_captures still applies only its gross-error warnings (3 mm, 2°), so a dedicated mount-check mode with the Step 4.8 tolerances is a code follow-up.

Forward note for C14 (Analysis D), found while making the zero-detection test honest (2026-10-05): with the
false-alarm target γ = 0.01 per window and 300 extended trials, a feature that is truly never detected still
collects about 3 false detections; the one-sided 95% Clopper–Pearson upper bound on 3/300 is about 2.6%, and the
corrected bound (ψ_U − γ)/(1 − γ) ≈ 1.6% exceeds DETECTION_ZERO_PROBABILITY_BOUND = 0.01. The bound is met only
when at most 1 of the 300 trials fires, which happens about one time in five. So the 0% criterion of Section 13,
Step 6 cannot be met reliably as specified. Options to put to Neil in C14: raise the bound to about 0.02 or
0.03, lower the false-alarm target for the extended trials, raise the trial count, or define the 0% point as
"not distinguishable from the blank sites" with a two-sample bound. Also a code follow-up (C04-R3): derive the
nominal post diameter from POST_DIAMETER_FRACTION_OF_D0 × the expected D_0 instead of a fixed 0.5 mm.

## C06 — Section 5 Acquisition A, noise-plate series (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C06-R1 (serious, geometry): the tilt sub-series at the reduced stations includes Z_MIN = 400 mm with tilts up to
  45°. A 400 mm plate tilted 45° about its center at 400 mm puts its near edge at about 259 mm, inside the sensor's
  near limit (Z_MIN is the dagger-marked limit of valid depth), and its far edge at 541 mm; the plan tool already
  warns that the tilted plate fits worse. Proposal: tilt only where the tilted plate's near edge stays at or beyond
  Z_MIN (near edge = Z − half-plate × sin tilt), which the planner checks; at the indicative plate size that drops
  400 mm and leaves 800 and 1600 mm. If a near tilt station is wanted, add 566 mm, where 45° keeps the edge at 425 mm.
- C06-R2 (serious, procedure cost): a drift sentinel is defined on T2, so each of the 8 sentinels that fall during B,
  C and D forces a re-mount of T2 (the plan tool reports it), which costs time and injects the re-mount error it is
  meant to watch. Proposal: define the sentinel as the front plane of whatever target is mounted, captured centered
  at Z_REFERENCE_MM, with the first sentinel after each mount as that target's reference; T2 sentinels remain at
  the series boundaries. Decision for Neil (changes Sections 5, 7, 8, 9 and the plan tool).
- C06-R3 (from C05-N1): "check that the plate fully covers the analysis region of interest" has no rule. The plan
  tool places the plate as far off-axis as keeps it inside the field with a margin and logs the achieved fraction
  (36 % of the requested offset at 476 mm). Proposal: state that rule, with the margin BOUNDARY_BAND_HALF_WIDTH_PX,
  and require the achieved field fraction to be logged per station and reported with the A results.
- C06-R4 (budget): the legacy stations 700 and 1000 mm are captured at all five field positions, but the legacy
  metrics use the center only. Proposal: legacy stations at the center only (A drops from 80 to 72 poses; the
  Section 9 budget and the plan tool change accordingly).
- C06-R5 (low): Step 4 logs the sensor temperature; the air temperature comes from the loggers of Step 4.1, say
  so; Step 8's "Steps 1–5" should include Step 6 (the repeat-mount check is part of the filters-off repeat).

Neil's comments: (pending)
Neil's comment (2026-10-05) on C06: Section 5 would be very difficult to follow as a procedure; imperatives such
as "Build the station list from every station..." carry no definitions or guidance and a naive reader could not act
on them. Neil hopes the data acquisition (technician) document is clearer. Logged as C06-P1. Reviewer
response pending: compare with the technician procedure's Sections 5 and 6 and propose how the specification should
define its terms (station, field position, pose list) and point to the plan tool's pose list as the thing the
technician executes.
- C06-P1 reviewer response: the technician procedure (docs/procedures, Sections 5 and 6) is the executable form: run
  the planner, follow poses.csv in `order`, the robot program outline, file names, pose log, mount check, sentinel
  handling, and what to tell the engineer. The specification's Section 5 states what the series contains and assumes
  the plan tool. Proposal C06-R6: add a lead paragraph to Part I defining the terms every series uses (station = a
  commanded target-center Z on the ladder; field position = center or one of four corners at FIELD_OFFSET_FRACTION
  of the half field; pose = one commanded target position and orientation, turned into a flange pose by the
  registration; pose list = the plan tool's poses.csv, which the robot program executes in order) and stating the
  division of labor: the specification says what each series captures and why, the technician procedure says how.
  Then rephrase the Section 5 imperatives against those terms ("The pose list for A holds every ladder station ...").
- C06-P2 (Neil, 2026-10-05): the specification skirts the line between a human-readable document and a computer
  specification, with many shibboleths (the code-style parameter names and constructs in the prose); the plan
  documents (technician procedure Sections 5 and 6) are good. Style decision pending: how the prose should refer to
  parameters (plain words with the value, the code name only in the Section 2 table and at first mention).
C06 closed 2026-10-05 ("closed."; R2 "accepted as written"). Executed (document rev 84): R1 tilt rule (near edge
≥ Z_MIN; 400 mm skipped, 800 and 1600 kept); R2 sentinels on the mounted target's front plane with the first
sentinel after each mount as its reference, T2 sentinels bracketing each series (Sections 5, 7 and 10 Step 11);
R3 coverage rule (pulled inward with the BOUNDARY_BAND_HALF_WIDTH_PX margin, achieved fraction recorded and
reported); R4 legacy depths at the center only; R5 air temperature from the loggers, filters-off repeat covers
Steps 1–6; R6 Part I lead paragraph defining station, field position, pose and pose list and the division of
labor with the technician procedure; Section 5 rewritten against those terms in a plain-language style (values in
prose, the parameter name once in parentheses) as a sample for the P2 style decision. Code: planner changes
delegated (sentinel target, tilt rule, legacy center, achieved fraction); the Section 9 budget row for A follows.

## C07 — Section 6 Acquisition B, edge-target and Z-step series (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C07-R1 (serious, design): the Z-step ladder is fixed in millimeters (0.1 to 4 mm) while the expected depth
  quantum grows as Z²: at the indicative geometry (q = 0.125 px, k = f_x·B ≈ 51,600 px·mm) δZ_q is 0.39 mm at
  400 mm, 1.55 mm at 800 mm and 6.2 mm at 1600 mm. At 1600 mm the whole ladder lies below one quantum, so δ_50
  cannot be bracketed there; at 400 mm the 0.1 mm rung is a quarter quantum. Proposal: define the rungs as multiples
  of the expected quantum at Z₀ (for example 0.25, 0.5, 1, 2, 4, 8 × δZ_q, the Tier-A q until A measures it, as
  the staircase already does), with a floor of 2 × ROBOT_REPEATABILITY_MM so the truth ratio never exceeds 50 %;
  the plan tool computes the millimeter rungs per station. Budget unchanged (6 rungs).
- C07-R2 (serious, consistency): the edge series says nothing about the approach direction, although Neil
  asked earlier whether the approach discipline should cover every series; lateral backlash shifts the true edge
  relative to the commanded pose. Because the read-back pose is logged and Section 11 projects the true edge from
  the registered read-back pose, the shift is harmless only if that is what Section 11 uses. Proposal: state in the
  Part I lead that every pose of every series is approached along the same direction (from below in Z, from −H and
  −V laterally), and in Section 11, Step 3 that the true edge line is projected from the read-back pose carried
  through the registration, never from the commanded pose.
- C07-R3: the slant EDGE_SLANT_DEG and the as-built gap of each spacer set must be measured, not nominal; the
  as-built record of Section 3.3 lists neither. Proposal: add both to the as-built record and say so in 6.1, Step 1.
- C07-R4 (low): 6.1, Step 2 "centered" at 1600 mm gives a 69 px square with 8 px bands and 8 px jitter, which is
  enough; say that the plan tool checks the square's size against the band and jitter at every shape station.
- C07-P2 (style, pending): this section is the densest in parameter names; it is the next candidate for the
  plain-language style once Neil decides.

Neil's comments: (pending)
- C07-P1 (Neil, 2026-10-05): instead of moving the target in Z by multiples of a suspected quantum, tilt the target
  by a small angle about H and/or V so that the true depth varies smoothly across the plate; one capture then shows
  the quantization plateaus along the ramp, and the many small Z moves are not needed. Reviewer assessment given in
  chat (adopt as a ramp sub-series; geometry table; fixed-pattern subtraction from A; one time-staircase kept at the
  reference station as a cross-check is the open question).
- C07-P1 decision (Neil, 2026-10-05): replace the fine staircase entirely with the tilted-ramp probe; Neil does not
  believe the robot can resolve Z motions smaller than 0.1 mm. Consequences to execute at the close of C07: Section
  6.2 Step 3 becomes the ramp sub-series (T2 tilted about H at every ladder station by the angle that spans
  RAMP_QUANTA = 4 expected quanta across the visible plate, FRAMES_PER_RAMP_POSE = 50, read-back pose logged);
  Section 2 drops Z_STAIRCASE_QUANTA, Z_STAIRCASE_SUBDIVISION and Z_STAIRCASE_FRAMES and gains RAMP_QUANTA and
  FRAMES_PER_RAMP_POSE; Section 10 Step 8 points to the ramp instead of the staircase; Section 11.2 Step 5 becomes
  the ramp analysis (fixed-pattern map from A subtracted, row averages against the true depth per row from the
  read-back pose, plateau widths = quantum, single-pixel curve against the row average for dithering); Section 9
  budget B-Z row changes; the Z-step ladder floor stays at 0.1 mm (R1). Code: planner, simulator, resolution_depth
  analysis, tests, technician procedure.
- C06-P2 decision (Neil, 2026-10-05): carry the plain-language style throughout the document unless it makes the
  meaning unclear. Style rule: in prose, say the quantity in words with its value, and give the parameter name once
  in parentheses at its first mention in a section; the Section 2 table, equations and code keep the names.
  Applied to every chunk from C07 on as it is executed; Sections 1, 3 and 4 (already closed) get a style pass with
  the gate at the end of the review, declared as intended.
- C07-P1 addendum (Neil, 2026-10-05): if the measured quantum is 0.1 mm or larger, the staircase may be used as a
  second-pass option. Reviewer note: with the staircase step at one tenth of the quantum, a 0.1 mm quantum would
  ask for 0.01 mm moves, below the 0.1 mm the robot resolves; so the optional staircase keeps its step at the larger
  of quantum / Z_STAIRCASE_SUBDIVISION and a new ROBOT_MIN_RESOLVABLE_MOVE_MM = 0.1 (Neil's figure), which makes it
  useful where the quantum is at least about 1 mm (800 mm and beyond at the indicative geometry) and a coarse but
  valid temporal cross-check elsewhere. Section 6.2 therefore keeps the staircase as an optional second pass, run
  only after the ramp has measured the quantum, outside the main budget; the Z_STAIRCASE_* parameters stay, marked
  optional.
- Neil's question (2026-10-05, during C07 close-out): would anything be learned by moving a plate in small lateral
  increments (H and V, perpendicular to its normal) while mapping the reported against the actual location of its
  edge pixels? Reviewer answer in chat: the random jitter poses already contain those pairs, so an "edge position
  transfer" analysis (reported edge crossing s_50 per pose against the read-back lateral offset: lateral gain,
  1-px-periodic pixel-locking bias, dot-pitch periodicity, approach hysteresis) can be added to Section 11.1 at no
  capture cost; a systematic fine sweep (0.1 px steps over 2 px, in H and in V, at the reference station) is an
  optional second pass that resolves the shape of the periodic bias better. Proposal carried to C12 (Analysis B).

C07 executed (2026-10-05; document rev 86 to 88, recorded here after the fact): R1 ladder rungs as quanta multiples
(Z_STEP_LADDER_QUANTA 0.25 to 8) floored at ROBOT_MIN_RESOLVABLE_MOVE_MM = 0.1 mm; R2 approach direction stated in
the Part I lead and the read-back pose as the edge truth in Section 11; R3 measured slant and gap in the as-built
record; R4 plan-tool check of the square's size; P1 the ramp sub-series in 6.2 Step 3 (RAMP_QUANTA = 4,
FRAMES_PER_RAMP_POSE = 50) with the staircase as the optional second pass (Z_STAIRCASE_* kept, marked optional);
Section 2, 9, 10 and 11.2 follow; plain style applied. Code landed 2026-10-05 (commit 0ac4a29): 9 ramp poses,
budget 7,194 / 42,520 / 7.18.

## C08 — Section 7 Acquisition C, disk and cutout area series (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C08-R1 (clarity, as C06-P1): Step 1 "Build the configuration list" is an imperative with no actor. Proposal:
  rewrite the section in the pose-list, plain-language style: the pose list for C holds, for each plate (T4, T5)
  and each gap (15 and 60 mm), every station of the ladder, centered and fronto-parallel; the plan tool randomizes
  the order within each mounting; mount each plate with the mount check of Step 4.8.
- C08-R2 (feasibility): the open-background variant requires "nothing within the sensor's range behind the holes";
  with the range now 1600 mm and the variant at 800 mm, that is at least 800 mm of clear space behind the plate, or a
  surface beyond the sensor's range. Proposal: state the clearance (Z_MAX − Z_REFERENCE_MM) explicitly.
- C08-R3 (consistency with Section 5): the field sub-series requests the four off-axis positions at 0.6 of the half
  field, but the 336 × 198 mm plates do not fit there at 800 mm; the plan tool pulls them inward (68 to 96 % of the
  requested offset at the indicative geometry). Proposal: say that the rule of Section 5, Step 1 applies and that the
  achieved fraction is logged and reported with the C field comparison.
- C08-R4 (low): Step 2 should give the values (30 poses, ±4 px of the 8 px span, 10 frames) in the plain style;
  Step 5's sentinel sentence is now consistent with Section 5 and needs no change.

Neil's comments: no additional comments on C08 (2026-10-05). On the edge position transfer analysis: place it in
the test plan where it is most efficiently executed, and in the specification wherever it is most logical given the
document's structure.
Executed: R1–R4 (Section 7 rewritten in the pose-list, plain-language style; open-background clearance stated as
Z_MAX − Z_REFERENCE_MM; field sub-series pull-in and achieved fraction stated). Edge position transfer: the analysis
goes into Section 11.1 at C12 (it uses the B jitter poses, no capture cost); the optional fine lateral sweep is
added now as Section 6.1, Step 6 (optional second pass, LATERAL_SWEEP_STEP_PX = 0.1 px, LATERAL_SWEEP_SPAN_PX = 2 px,
rows added to Section 2), outside the budget; in the technician procedure it belongs to Series B as an optional
step after the edge captures, where the target is already mounted.

## C09 — Section 8 Acquisition D, detection trials (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C09-R1 (serious, statistics; the C14 forward note lands here): Step 3 says "300 trials bound a never-detected
  level at ≤ 1 %". That holds for the raw rate only. After the false-alarm correction of Section 13, Step 6, a
  feature that is never detected still fires about 3 of 300 windows at the 1 % false-alarm target, and the corrected
  95 % upper bound is then about 1.6 %, above the 1 % bound; it passes only when at most 1 window fires, about one
  time in five. Distinguishing "never detected" from the 1 % blank rate to within 1 % needs about 540 trials
  (one-sided 95 %, two-sample). Options for Neil: (a) keep 300 trials and set DETECTION_ZERO_PROBABILITY_BOUND to
  0.02, with the 0 % criterion defined as the one-sided 95 % bound on the excess of the feature's rate over the
  blank-site rate (two-sample), which 300 trials resolve to about 1.3 %; (b) keep the 1 % bound and raise
  DETECTION_ZERO_TRIALS to 600 (the D extended row roughly doubles, about +2.5 h); (c) lower the false-alarm target
  for the extended trials, which the blank sites cannot calibrate at 300 trials (a 0.1 % quantile from about 900
  blank windows). Recommendation: (a). Sections 2, 8, 9 and 13 change with it.
- C09-R2 (budget consistency): Step 5 lets the first frame of each C pose count as a D trial (30 of the 60 per
  configuration and station), but the Section 9 budget counts all 60 as new poses and the plan tool plans them
  all. Proposal: say that the budget assumes no reuse and that the plan tool applies the reuse on request
  (shortening D main from 2,160 to 1,080 poses), or drop the reuse. Recommendation: keep it as the stated option.
- C09-R3 (clarity, plain style): rewrite the intro and Steps 2–3 with the values (3 features, 9 stations, 60
  poses of 1 frame, 300 trials at the 3 farthest stations 1131, 1345, 1600 mm) and the parameter names once.
- C09-R4 (low): Step 1 is fine; Step 4 should also forbid reusing a C pose's later frames (only its first frame
  is independent of the D poses), which Step 5 implies.

Neil's comments (2026-10-05): R1: measure at 3–5 %; extrapolate the curve to 0 % as a clearly flagged prediction,
not a measurement. Closed.
Executed (document rev 91): the lowest measured point is D_5 (DETECTION_LOW_PROBABILITY = 0.05, which 300 trials
resolve to about ±2.5 %; 3 % would be marginal at that count); DETECTION_ZERO_TRIALS → DETECTION_LOW_TRIALS,
DETECTION_ZERO_STATION_COUNT → DETECTION_LOW_STATION_COUNT, DETECTION_ZERO_PROBABILITY_BOUND replaced by
DETECTION_LOW_PROBABILITY, new DETECTION_ZERO_PREDICTION_LEVEL = 0.01 (the level the fitted curve is extrapolated
to); Section 1 measurand row, Section 8 (plain style, R2 reuse stated as outside the budget, R3, R4), Section 9
row label and the time-short note, Section 13 heading, intro, Steps 6–9 and 11, Section 15 limitation, and the
flow-diagram label follow. Code (detection analysis, parameters, tests) and the technician procedure follow once
the ramp agent lands (shared parameters module).

## C10 — Section 9 Data logging, file layout, and capture budget (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C10-R1 (numbers): the budget table predates the C06, C07 and C09 changes (A at the legacy center only and the
  tilt rule, sentinels on the mounted target, the ramp in place of the staircase, the low-point series). Proposal:
  refill every row and the totals from the plan tool at close, after the ramp code lands, and keep the three
  totals as gate invariants.
- C10-R2 (consistency with the code): the manifest column list omits columns the code writes and the procedure
  relies on: the sub-series label (main, tilt, remount, nominal, jitter, ladder, ramp, field, extended, sentinel),
  tilt axis and angle, commanded step and visit (B-Z), the feature index, the achieved field fraction, and the
  sentinel's mount-reference flag. Proposal: list the columns as the manifest module defines them, in its order,
  and say that extra metadata columns may follow.
- C10-R3 (file names): say that the pose index has three digits, four for the optional repeats planned outside the
  budget, and that a target without a back plate writes G0.
- C10-R4 (budget scope): say which captures are outside the budget: the filters-off repeat, the optional
  staircase and lateral sweep, the open-background variant, the D reuse option, and that target swaps, warm-up,
  mount checks and registration time are excluded as before.
- C10-R5 (plain style): the folder-layout and manifest paragraphs are already plain; no change beyond R2–R4.

Neil's comments: none; closed 2026-10-05.
Executed (document rev 92): R2 manifest columns listed as the manifest module writes them; R3 file-name digits,
G0 and f00 stated; R4 the budget's exclusions and the optional captures listed. R1 (the table numbers) waited for
the ramp code; executed at document rev 93 once it landed: rows refilled from the plan tool (A main + tilt 64 poses,
5,600 frames, 0.21 h; B-Z ladder + ramp 369, 4,050, 0.42; a separate Sentinels row 11, 330, 0.02; totals 7,194
poses, 42,520 frames, 7.18 h), gate invariants updated, gate PASS with scope ['9'].
- Follow-up for Neil (from the ramp code, 2026-10-05): at 400 mm the ramp's tilt of 0.318° would put the plate's
  near edge 1.1 mm inside Z_MIN, so the planner moves the plate center 1.11 mm farther (station label stays 400 mm,
  shift recorded in the pose notes). Section 6.2 does not mention this; proposal: one sentence in 6.2, Step 3
  (closed chunk C07; to be declared as intended when executed). Also noted by the code: the far edge of the ramp
  at 1600 mm lies 12 mm beyond Z_MAX, and the 49.6 mm rung reaches 1650 mm.

C10 addendum (2026-10-05): the storage sentence of Section 9 said "about 44,000 frames"; now "the 42,520 frames of
the table" (rev 94, gate PASS, scope ['9']). The technician procedure (docs/procedures, baseline 2026-10-05_12) and
the code (commits ad5ebdc and later) now carry C06 to C10. Inconsistencies between the specification and the code
found by that pass, logged as follow-ups for Neil rather than edited silently:
- F1 (sentinels at series boundaries): Section 5 says T2 sentinels bracket each series; the code captures the
  boundary sentinel on the target that is mounted at the time (T2 before and after A and after B-Z; T3b after B;
  T5 after C and after D), so no extra T2 mount is needed. Recommendation: align the specification to the code,
  with T2 at the session's start and end and wherever it is mounted anyway, because every target's drift chain is
  continuous within its mount and the session-level chain is carried by T2 at A and B-Z.
- F2 (approach direction): the Part I lead says every pose is approached from below in Z and from −H and −V; the
  planner records the approach only for B-Z (from below) and the lateral sweep (alternating). The robot program
  applies the rule; a code follow-up could record the approach for every pose.
- F3 (coverage margin): Section 5, Step 1 states the 8 px margin only; the planner adds half the phase-jitter span
  for the jittered series. Proposal: add that clause to Section 5, Step 1 (closed chunk; declare as intended).
- F4 (staircase threshold): Section 6.2 says the optional staircase is useful where the quantum is at least about
  1 mm; Neil's rule allows it from 0.1 mm. Both hold (allowed from 0.1 mm, well resolved from about 1 mm); the
  procedure uses 0.1 mm. The measured quantum is passed to the planner through the parameters override.
- F5 (code, done, commit 7249923): the sentinel mount-reference flag is a manifest column as Section 9 lists it; the
  optional staircase (P2000+) and lateral sweep (P3000+) carry four-digit pose indices as Section 9 states; the Tier-A
  quantum is an overridable parameter so the ramp's measured quantum reaches the optional staircase.
- F6 (code follow-up): with --filters-off the main-budget totals read 7,196 poses and 42,580 frames because the
  repeat's two sentinels fall on the budget clock; the repeat itself is outside the budget. The sentinels of an
  optional set should be counted with that set.

## C11 — Section 10 Analysis A, noise vs Z (opened 2026-10-05)

Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C11-R1 (correctness): Step 4 takes the standard deviation of the residual of the frame mean about a free plane
  as σ_fp, but the mean of 100 frames still carries temporal noise of σ_t/√100, about 10 % of σ_t, which inflates
  σ_fp where the fixed pattern is small. Proposal: σ_fp² = var(residual) − σ_t²/N with N the frame count, stated.
- C11-R2 (consistency of the closure check): Step 5 checks σ_tot² ≈ σ_t² + σ_fp² + bias², but σ_fp is defined
  about a free plane while σ_tot and bias are about the registered plane; the tilt between the two planes (Step 4
  reports the angle) breaks the identity. Proposal: define σ_fp as the spread of (Z̄ − Z_GT − bias) about the
  registered plane, keep the free-plane fit only for the reported tilt angle, and cite the tolerance of 20 %
  (NOISE_CLOSURE_TOLERANCE) instead of "≈".
- C11-R3 (plain style and names): Steps 1, 7, 8, 12 carry names without values or values without names
  (8 px band, 1/e, 20 × 20 px, half side 10 px, the five box positions of the legacy script); rewrite under the
  plain-language rule.
- C11-R4 (consistency with C06): Step 10 says "at each Z" for the incidence-angle plots; the tilt sub-series now
  runs only where the tilted plate fits (800 and 1600 mm at the indicative plate size). Proposal: "at each tilt
  station".
- C11-R5: Step 7 should point to the jitter-span check of Section 5, Step 7 as the moment the comparison with
  PHASE_JITTER_SPAN_PX is made; Step 9 should say what weights the least squares uses (the standard error of
  each station's median σ_t).
- C11-R6 (outputs): Step 11 should report the drift rate per mounted target and flag any series whose drift
  exceeded the warm-up fraction; Step 13's summary gains the achieved field fraction per station (C06) and the
  per-target drift.
- Pipeline: the renderer restarted the step numbers after the formula (10–13 shown as 1–4); fixed by honoring the
  continued list's start number.

Neil's comments (2026-10-06): R1 correction term approved; R2 to R6 approved. Closed.
Executed (document rev 95; gate PASS, scope ['10', '5']): Step 4 σ_fp about the registered plane with the σ_t²/N
correction, free plane kept only for the angle; Step 5 closure with the 20% tolerance named; Steps 1, 7, 8, 12
in the plain style; Step 7 points to the Section 5, Step 7 quick-look; Step 9 states the 1/σ_t weights (what the
code does); Step 10 "at each tilt station"; Step 11 drift rate per mounted target and the flag; Step 13 field
fraction and drift rate per row, drift figure. Also executed in the same update, from Neil's decision on
follow-up F1 ("accept first option"): Section 5, Step 3 now says the boundary sentinel is captured on the target
mounted at that moment and T2 serves wherever it is mounted anyway, matching the code. Code (noise.py) and the
technician procedure Section 14 delegated. The renderer fix for continued numbering is in commit 218323a.

## Drift run (Neil's question during C11 close-out, 2026-10-06)
Neil asked about measuring sensor drift separately and omitting the sentinels. Reviewer assessment given in
chat: a separate static run measures the sensor alone (not the robot's thermal growth or mount events), the
sentinels cost under 1% of the session, and the drift-versus-temperature relation may not reproduce between
days; recommendation: keep the sentinels and add the separate run as an optional pre-session measurement.
Neil's decision: add the optional separate drift run to Section 4. Executed (document rev 97; gate PASS, scope
['2', '4', '9', '10']): Section 4, Step 3 gains the optional run (T2 on a fixed stand at the reference station,
robot idle, from cold, SENTINEL_FRAMES frames every DRIFT_RUN_CAPTURE_INTERVAL_MIN = 2 min for
DRIFT_RUN_DURATION_MIN = 480 min, temperatures logged, files under sentinels/ with the sub-series label
drift_run, outside the budget); Section 2 gains the two rows; Section 9 lists the run among the optional captures
excluded from the budget; Section 10, Step 11 compares each mount's sentinel drift with the drift predicted from
the run's temperature relation. The two values (480 min, 2 min) are reviewer choices open to change. Code and
technician procedure follow (noise analysis agent, then the procedure's Section 3/4).

## C12 — Section 11 Analysis B, resolution in H, V, Z (opened 2026-10-06)
Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C12-R1 (Neil's C07 question; placement agreed at C08): add an "Edge position transfer" step to 11.1 after Step 8:
  for every jitter pose of an edge, the pose's own edge crossing s_50(pose) against its read-back lateral offset
  carried through the registration; regress to get the lateral gain (slope, expected 1), then fit the residual
  with a 1 px periodic term (pixel locking) and a term at the projector dot pitch taken from the Section 10
  correlation length; report both amplitudes and the gain per Z and orientation. Where the optional lateral sweep
  of Section 6.1, Step 6 was run, add the approach hysteresis (mean difference of s_50 between the poses approached
  from the negative and the positive side) and the periodic bias at 0.1 px resolution. No new capture; code
  (resolution_lateral) and the technician procedure's Section 14 follow.
- C12-R2 (stale reference and rule, Step 10): "left out of the gain regression of Step 2" should read Step 11; and
  the truth rule "rungs smaller than ROBOT_REPEATABILITY_MM are flagged" is now never triggered, because the ladder
  floor is 0.1 mm = 2 × 0.05 mm by the C07 decision. Proposal: flag rungs smaller than twice the robot
  repeatability (the floor rule, so the truth ratio never exceeds 50%), which keeps the rule meaningful for a
  different robot.
- C12-R3 (serious, Step 14, from the ramp code): the row average shows plateaus only when the disparity noise is well
  below the quantum; at the indicative noise (about 0.6 q) the frames dither the quantizer and the time-averaged row
  curve is smooth at 400 and 800 mm, with plateaus resolved only at 1600 mm. The code classifies each curve as
  stepped or smooth (RAMP_MAX_INTERMEDIATE_FRACTION, RAMP_INTERMEDIATE_BAND) and, when the average is smooth, takes
  the quantum from the spacing of the populated depth levels of the pooled readings (Section 10, Step 8's method on
  the ramp data), reporting which method gave it. Proposal: state this in Step 14, with the names, so that a smooth
  ramp curve is read as a finding (dithering) rather than a failure, and the quantum still comes out.
- C12-R4 (plain style): Steps 1, 4, 7, 9, 10, 12, 13 carry names without values or values without names: 8 px band,
  0.25 px bins, 15 and 60 mm gaps, 0.1 in h (ESF_LINEARITY_TOLERANCE_H, unnamed in the text), 2000 resamples,
  0.05 mm repeatability, 1% false-alarm target, patch sizes 1, 5 × 5 and 20 × 20 px (ZSTEP_PATCH_SIZES_PX, unnamed).
- C12-R5 (outputs): Section 11 has no Outputs step while Sections 10 and 13 have one. Proposal: add Step 15 listing
  B_resolution_summary.csv (one row per edge, Z, orientation, polarity, gap), the Z-step summary and rungs CSVs,
  Z_ramp_rows.csv, the figures (ESF curves, LSF and MTF, rise vs Z, s_50 vs Z, Z-step detection curves, ramp,
  quantum vs Z), and the forward-model terms (rise H, rise V, s_50) that go to forward_model_parameters.json.
- C12-R6 (sign convention, Step 8): say what the sign of s_50 means (negative: the measured edge lies on the
  back-surface side, that is the front surface is fattened), and that Section 14 reports s_50 beside the near/far
  preference index rather than "uses it as" that index.
- C12-R7 (low, Step 5): "once aligned" leaves the alignment undefined; the code aligns the two ESFs by their s_50
  and reports the residual as an equivalent shift. Say so.
Neil's comments (2026-10-06): R1 to R7 approved. Closed.
Executed (document rev 102; gate PASS, scope ['11', '2', '6']): R1 new Step 9 "Edge position transfer" (gain,
pixel-locking and dot-pitch periodic terms, hysteresis from the optional sweep); R2 Step 11 (was 10) flags rungs
below twice the repeatability and points to the gain regression of Step 12; R3 Step 15 (was 14) states the
dithering condition, the stepped-or-smooth rule with RAMP_MAX_INTERMEDIATE_FRACTION = 0.5 and
RAMP_INTERMEDIATE_BAND = 0.25 to 0.75 of a quantum (two Section 2 rows, marked chosen by argument), and the
depth-level fallback; R4 plain style in Steps 1, 4, 7, 10, 11, 13; R5 new Step 16 Outputs; R6 sign of s_50 and
"beside the near/far index"; R7 ESF alignment at the half-height crossings with the equivalent shift. Section 6.2's
ramp item gains "or from the depth levels where the noise dithers them away". The 11.2 list now starts at 11 and
the two cross-references were updated. Code (resolution_lateral, resolution_depth) delegated; technician
procedure Section 14 follows.
C12 addendum (rev 104, gate PASS, scope ['11']): the code pass showed that s is measured from each pose's own true
edge, so an ideal sensor gives a constant s_50 and the lateral gain is 1 plus the regression slope, in a joint fit
with the periodic terms; Step 9 now says so. Step 11's parenthetical now says the 2 × repeatability rule equals the
0.1 mm ladder floor at the indicative values (ROBOT_MIN_RESOLVABLE_MOVE_MM stays Neil's independent figure). Code
landed in commit 8f7bfc4 (199 tests); approach_direction becomes a manifest column in a follow-up.

## C13 — Section 12 Analysis C, true vs sensed area (opened 2026-10-06)
Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C13-R1 (Step 9, scaling test): "agreement within the bootstrap interval" does not say what is tested. The code
  computes, for each pair of neighboring features and each ratio, the mean difference of the two transfer curves
  over the shared D_px range and whether zero lies inside its bootstrap interval, one row per (kind, gap, ratio) in
  C_overlap_test.csv. Proposal: state that test and that file.
- C13-R2 (Step 7, projector light): "it is not known whether the VSX3000 needs projector light on a point to read
  it" conflicts with the detection model behind the 7 px expected minimum (active stereo matches the projected
  pattern; a point without projector light has no texture to match). Proposal: say the three-center version is
  the expected one for that reason and the two-camera version is the check, and report which tracks the data.
- C13-R3 (Step 11, method): the code cannot do "blur with the H and V line spread functions" as written, because
  Section 11 hands over rise distances, not the LSF; it uses a Gaussian whose 10–90% rise equals the rise
  distance, and makes the prediction twice (pure blur-and-threshold, and with the edge offset s_50 applied as a
  growth of the front material, because a symmetric blur cannot shift the 0.5 crossing). Also the sign relation is
  not stated: b_disk ≈ −s_50 and b_cutout ≈ +s_50. Proposal: require Section 11's outputs to include the LSF
  (Step 16) and Step 11 here to use it, with the Gaussian of equal rise as the fallback; state the two predictions
  and the sign relation. Code follows (resolution_lateral writes the LSF; area reads it).
- C13-R4 (Step 10, fit rule): "the largest feature, well above D_50" needs the rule the code applies: the largest
  feature at the stations where its D_px is at least a named threshold (AreaOptions.bias_fit_min_d_px, not in
  Section 2). Proposal: name it AREA_BIAS_FIT_MIN_D_PX in Section 2 with its value and use it here.
- C13-R5 (plain style): Step 1 "8 px band"; Step 3 the 0.5 threshold named (FRONT_READ_HEIGHT_THRESHOLD); Step 4
  "grown by one pixel" (a code constant, fine as words); Step 5 the h = 0.5 contour (ESF_HALF_HEIGHT); Step 8 the
  phase spread over the 30 jitter poses (PHASE_JITTER_POSES_AREA) and the temporal spread over the 10 frames
  (FRAMES_PER_AREA_POSE); Step 1 add that a feature whose window leaves the image is skipped and counted.
- C13-R6 (Step 12, low): the field comparison carries the achieved field fraction of each pose (Section 7) as a
  factor; the open-background comparison is at the reference station only.
- C13-R7 (Step 13, outputs): list as Section 11 does: C_area_summary.csv (one row per feature, station, gap,
  field position and sub-series), C_overlap_test.csv, C_area_details.json; figures: transfer curves with the
  overlap ranges shaded, b against Z, phase spread against D_px, predicted against measured area. No term of C
  enters forward_model_parameters.json; b is a cross-check of the boundary widths of Section 14.
Neil's comments (2026-10-06): R1 to R7 approved; closed. Note on R2: "projector light" is ambiguous between
visibility for the depth sense (both IR cameras at 0 and B, with the laser dot projector midway between them) and
visibility for the 2-D information (a 2-D sensor on the axis between the IR sensors with collocated illumination).
Executed (document rev 106; gate PASS, scope ['2', '11', '12']): R1 Step 9 states the overlap statistic (mean
difference over the shared range, zero inside its bootstrap interval) and C_overlap_test.csv; R2 Step 7 states the
depth-sense visibility rule (both cameras see the point and the projector lights it), that with the projector
midway the three-center overlap equals the two-camera one for a convex outline, that both are computed with the
as-built projector position, and that the 2-D image's visibility is a different, uncharacterized quantity; R3 Step
11 uses Section 11's line spread functions (new B_lsf.csv in Section 11, Step 16) with the Gaussian of equal rise as
the fallback, the two predictions, and the sign relation; R4 Step 10 names AREA_BIAS_FIT_MIN_D_PX = 14 px (twice
the 7 px expected minimum; new Section 2 row; the code's 4 px threshold changes); R5 plain style in Steps 1, 3, 4,
5, 8 and the skip rule; R6 Step 12 field fraction as a factor and the reference station; R7 Step 13 outputs listed.
Code (area, resolution_lateral) and the technician procedure's Section 14 delegated.

## C14 — Section 13 Analysis D, minimum detectable size (opened 2026-10-06)
Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C14-R1 (serious, Step 3, threshold calibration): at the main stations each blank site has 60 trials, so the 99%
  quantile that sets τ is in practice the maximum of 60 blank windows (the code's "higher" quantile does exactly
  that and says so). The maximum of 60 samples lands near the 98.4% point on average with a wide spread, so the
  realized false-alarm rate at the main stations is poorly controlled; only the 3 extended stations (300 trials)
  resolve a 1% quantile. Proposal: pool the blank windows of a station over its blank sites (3 × 60 = 180) and,
  where the pooled count is below a named minimum (DETECTION_MIN_BLANK_WINDOWS, 300 = 3 / target), pool the
  neighboring stations after dividing the window statistic by σ_tot(Z) of Section 10; state that γ's
  Clopper–Pearson interval is reported with τ so the reader sees the control achieved. Code change.
- C14-R2 (Step 4 and 5, which fit gives which minimum): Step 4 fits the logistic; Step 5 fits three shapes and takes
  D_10 from the best by deviance; Steps 6 and 7 read D_5 and D_0 from "the fitted curve". Say once that D_50,
  D_10 and D_5 come from the best shape by deviance, with the range across shapes as model uncertainty for each.
- C14-R3 (Step 8, consistency with Section 12, Step 7 as revised): "all the views a read needs" is now defined
  there (both IR cameras and the projector); cite it.
- C14-R4 (Step 10 and 11, scaling test and outputs): the detection scaling test writes D_overlap_test.csv (one row
  per pair, kind, gap, field and rule); Step 11 should list it, plus D_pooled_summary.csv (the pooled rows) and
  D_detect_details.json, as the other sections now do, and say that D_50 and D_10 in px go to
  forward_model_parameters.json.
- C14-R5 (plain style): the intro and Steps 1, 2, 3, 4, 6, 7, 9 carry names without values (3 px window margin,
  2 connected pixels, 1% false-alarm target, lapse rate bound 0.05, 95% confidence, the 1% prediction level,
  2000 resamples, 8 px jitter span, 30 px isolation).
- C14-R6 (Step 2, the rule versus the sensor's floor): the 2 connected candidate pixels of the detection rule are
  the analysis's threshold for calling a trial detected, deliberately permissive; the sensor's own floor (the
  expected minimum of about 7 px from the laser-pencil model of Section 3.2) must come out of the data, not the
  rule. Add one sentence saying so.
- C14-R7 (cross-reference, my error at C13): Section 12, Step 10 cites "the 7 px expected minimum detectable
  diameter of Section 13"; the expectation is stated in Section 3.2. Fix the citation (Section 12, declared as
  intended) and have Section 13's intro name the Section 3.2 expectation as the prior the fitted D_50 and the
  predicted D_0 are compared with.
Neil's comments (2026-10-06): R1 to R7 approved; closed.
Executed (document rev 108; gate PASS, scope ['2', '12', '13']): R1 Step 3 pools a station's blank windows over
its sites, names DETECTION_MIN_BLANK_WINDOWS = 300 (new Section 2 row) below which neighboring stations are pooled
in units of σ_tot(Z), and reports γ's interval beside τ; R2 the intro and Step 5 say D_50, D_10 and D_5 come from
the best shape by deviance with the range as model uncertainty; R3 Step 8 cites the depth-read views; R4 Step 10
names D_overlap_test.csv and Step 11 lists the four files, the figures and the forward-model terms; R5 plain style
in the intro and Steps 1, 2, 3, 4, 6, 7, 9; R6 Step 2 says the 2-pixel rule is the permissive analysis threshold;
R7 Section 12, Step 10 now cites Section 3.2, and the intro names the Section 3.2 expectation as the prior. Code
(detection: τ pooling, best-shape minimums, prior comparison) delegated after the Analysis C agent lands, to avoid
concurrent edits of the parameters module.

## C15 — Section 14 Analysis E, boundary detection bias (opened 2026-10-06)
Reviewer findings (proposals; nothing executed until Neil closes the chunk):
- C15-R1 (Step 1, consistency with Section 12, Step 7 as revised): say that V is computed with the as-built
  projector position, that the depth-read rule (both IR cameras and the projector) is the primary one and the
  two-camera rule the check, and that the two coincide for a convex outline when the projector is midway.
- C15-R2 (Step 4, definitions): the rates are given by the formula block, but the widths are not defined;
  the code takes W_fab as the sum over bins of (reads at V = 0 in the bin / pixels in the bin) × bin width, an
  equivalent width, and W_drop likewise with no-reads at V = 1. State both so a reader can compute them.
- C15-R3 (Step 5, sign relation): the cross-check with s_50 and b has no stated relation; with the conventions of
  Sections 11 and 12, foreground fattening means π_near > 0, s_50 < 0, b > 0 for disks and b < 0 for cutouts.
  State it.
- C15-R4 (Step 7, intervals): the code bootstraps intervals for β_read and π_near only. Proposal: intervals for
  the two widths as well, from the same resamples (code change), and say the four quantities get intervals with
  2000 resamples.
- C15-R5 (Step 8, outputs and forward model): list E_boundary_bias.csv (one row per breakdown of Step 7 and per
  visibility rule) and E_boundary_details.json; figures (PNG and SVG); and the forward-model terms as the code
  writes them: W_fab, W_drop, π_near and β_read (the text omits β_read).
- C15-R6 (plain style): Steps 1, 2, 3 carry names without values: 8 px band, the 3 × σ_tot(Z)
  surface-assignment window (SURFACE_ASSIGNMENT_SIGMA_MULTIPLE), 0.25 px bins.
- C15-R7 (Step 6, low): say that the feature-scale classification uses the D trials (one frame per pose) and the
  C frames, so the fill-in fraction near D_50 has the same trial counts as Section 13.
Neil's comments (2026-10-06): R1 to R7 approved; closed. Note (on the 2-D visibility sentence of Section 12,
Step 7): the 2-D image's visibility should be computable from the light properties and the camera, light and
surface geometry.
Executed (document rev 110; gate PASS, scope ['2', '12', '14']): R1 Step 1 as-built projector position, primary
and check rules, the midway coincidence; R2 Step 4 equivalent-width definitions; R3 Step 5 sign relation; R4 Step 7
intervals for β_read, π_near, W_fab and W_drop with 2000 resamples (code change); R5 Step 8 files, figures and the
four forward-model terms; R6 plain style in Steps 1, 2, 3; R7 Step 6 trial counts. From Neil's note: Section 12,
Step 7 now computes the 2-D image's visible area A_geo_2D as the one-center projection from the 2-D sensor's
as-built position with its collocated illumination (new Section 2 row CAMERA_2D_OFFSET_MM, from the datasheet),
reported beside A_geo, and says that brightness follows from the light's properties and the camera, light and
surface geometry (cosine over squared distance, times reflectance), which the procedure records but does not
characterize. Code (boundary intervals, A_geo_2D) delegated after the Analysis D agent lands.

