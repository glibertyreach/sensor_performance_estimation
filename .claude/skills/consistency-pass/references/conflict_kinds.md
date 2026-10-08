# Kinds of conflict, one worked example each

Each example gives a governing statement, a derived statement, and why the second is
a conflict even though it reads sensibly on its own.

## Added option

Governing: "Material: finish-ground aluminum tooling plate or float glass."
Derived (suppliers table): "McMaster-Carr MIC-6 plus a grinding shop; Edmund Optics
float glass; a small granite surface plate."
Why: the governing set is closed ("A or B"); granite is a third member. A supplier
list is a set of claims that each entry meets the specification. Fix either side,
but report it: the user may want granite allowed, which changes the specification.

## Dropped condition

Governing: "Flat to 0.05 mm, checked after painting and after mounting."
Derived (deck caption): "Flat to 0.05 mm."
Why: the condition says when the check is valid; a plate flat before painting may not
be flat after. Before reporting, read the slide's notes: if they carry the condition,
the slide is a shortening, not a drop.

## Changed number or unit

Governing: "Free at least 25 GB on the capture computer."
Derived (checklist): "10 GB free."
Why: the derived figure is the per-1,000-frames rate, not the session total; a reader
who frees 10 GB runs out. Presentation differences ("25 GB" against "24 GB at 10 GB
per 1,000 frames") are not conflicts.

## Different party

Governing: "Flatness is confirmed in-house with the straightedge after mounting."
Derived (suppliers note): "Flatness checked by the supplier."
Why: who performs the check decides whether it happens; the supplier cannot check
after mounting.

## Reordered or merged steps

Governing: "Build the manifest, then run the check, then start the second run."
Derived (flow slide): "Run both sub-procedures, then build the manifests and check."
Why: the order exists so that flagged poses are re-captured while the setup stands.

## Provisional stated as settled

Governing: "Accept within 1 degree and 1 mm; these thresholds are placeholders until
the first session gives real figures."
Derived (slide): "Acceptance: 1 degree and 1 mm."
Why: a reader of the slide will treat the figure as a requirement and may reject good
data. The reverse also counts: a settled value presented as "to be confirmed".

## Stale value after an edit

Governing (after a revision): "Board at least 6 mm thick."
Derived (introduction written earlier): "6 mm instead of the stage-1 board's 15 mm."
Why: the derived text was true before the governing document changed; nothing
rebuilt it. This is the commonest kind and the reason to run the pass after every
edit to a governing document.

## Not a conflict: an unattached statement

Derived (notes): "MIC-6 is sold with a mill certificate."
Why: the governing document says nothing about certificates. List it under
"not attachable"; propose a governing sentence only if the user would want one.
