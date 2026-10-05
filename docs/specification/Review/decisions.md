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

Person's comments: (pending)
