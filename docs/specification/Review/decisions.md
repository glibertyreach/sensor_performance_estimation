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
