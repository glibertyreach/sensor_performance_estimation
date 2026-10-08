# Handoff: the sphere decision, from the VSX3000 performance-testing project to the binocular depth sensor calibration project

Date: 2026-10-08. Written for the calibration project's discussion so that it can be brought up to date without
reading the performance-testing session. Decisions are G Neil Haven's; the reasoning and the numbers are the
reviewer's (Claude), with their uncertainty stated.

## The three related projects

| Project | Repository | Role |
|---|---|---|
| Binocular depth sensor calibration (stage 1) | glibertyreach/depth_calibration_from_spherical_target, branch claude/vibrant-bardeen-160g78 | Depth correction map from spheres and a flat board; two decks; drawings SC1-01 to SC1-06 |
| Sensor registration via plane correspondence | glibertyreach/plane_plane_registration | Camera-to-robot registration from the stage-1 board; reuses the board, its adapter SC1-05 and the run-out fixture |
| VSX3000 performance testing (this project) | glibertyreach/sensor_performance_estimation, branch sensor-performance-procedure | Noise, resolution, area fidelity, detectability and boundary bias from robot-held knife-edge targets; specification in Claude Docs (revision 114 at the time of writing) |

## Decision 1: one sphere, the larger one

Neil decided (2026-10-08) that one sphere is sufficient to measure the curvature-dependent depth bias, and chose
the larger one: sphere B of the calibration project, 152.4 mm nominal diameter, turned aluminum with a matte
hard-anodized surface, M20 interface at a pole (drawing SC1-04 for the stem, SC1-02 for its adapter plate).
Sphere A (76.2 mm) and its drawings SC1-01 (adapter plate) and SC1-03 (stem) are no longer needed.

Why one sphere is enough. The sensor's spatial averaging acts in pixels, so the bias it causes on a curved
surface depends on the curvature of the depth profile in measurement space, not on the physical radius alone.
Over a sphere of radius R at distance Z, with focal length f in pixels, the depth profile in pixel coordinate s is
about z(s) = z0 + s² Z² / (2 f² R), so the measurement-space curvature is Z² / (f² R). R and Z enter only through
that combination. Sweeping one sphere through a Z range of 400 to 1600 mm changes it by a factor of 16, more than
a decade, whereas a second sphere of twice the radius shifts it by a factor of 2, inside the range the sweep already
covers. Within one frame the local curvature and slope also vary from the pole to the limb, so one capture samples
a continuum of measurement-space curvatures; the sweep adds the overall scale.

Why the larger sphere. The apparent radius shrinks with Z. At the indicative focal length of 688 px (unconfirmed),
the 76 mm sphere subtends about 65 px in radius at 400 mm and 16 px at 1600 mm, where the kernel covers a large part
of the sphere and boundary effects take over from the curvature term. The 152 mm sphere stays above about 33 px over
the same range.

What is given up. The scaling check that two spheres would provide (sphere A at Z and sphere B at 2Z present the same
measurement-space curvature; agreement between them tests that this is the governing variable). Neil accepted that
loss. A second caveat stays as a finding rather than a loss: if any of the sensor's processing works in millimeters
(a hole-filling or smoothing step with a millimeter threshold), the bias at fixed measurement-space curvature would
show a residual Z dependence, which the sweep reveals.

Expected magnitude, for scale. With the kernel's mean-square width in pixels written as ⟨s²⟩ and the pixel pitch
p = Z/f, the curvature bias at the pole is about p² ⟨s²⟩ / (2R). For a kernel about 2 px wide at 800 mm that is
roughly 0.035 mm on the 152 mm sphere (about 0.07 mm on the 76 mm sphere), against the calibration project's
0.1 mm target. The kernel width is unmeasured until the performance project's resolution series runs, so treat the
magnitude as uncertain to a factor of two.

## Decision 2: the sphere series runs in the calibration procedure

Neil decided that the curvature sweep is part of the calibration procedure, not of the performance-testing
procedure. Proposal for that project to adopt or adapt:

- Target: sphere B on its stage-1 stem and adapter plate, tool center point found as in the stage-1 procedure.
- Poses: the sphere center swept through a geometric ladder of distances, at the center of the field and at the
  off-axis field positions the calibration plan already uses. The performance project's ladder is 400 to 1600 mm at
  a ratio of 2^(1/4) (nine stations: 400, 476, 566, 673, 800, 951, 1131, 1345, 1600 mm). The calibration project's
  working volume is 300 to 1100 mm (its decision D-7); use that volume with the same ratio, or adopt the wider range
  if the sensor reads there (the performance project confirms its range limits before any series, its Step 4.5).
- Quantity measured: the depth bias of the sphere's surface against the known sphere (center from the tool center
  point, radius from the micrometer measurement), as a function of the local measurement-space curvature and slope,
  pooled over stations, with the per-station noise reported beside it.
- Cross-check: the prediction p² ⟨s²⟩ / (2R) from the line spread function measured by the performance project's
  resolution series (Analysis B writes it to B_lsf.csv), which is the only plane-based estimate of the same term.
- The curvature term of the correction map is then fitted from the sweep (one radius, many measurement-space
  curvatures) with the board supplying zero curvature, which is consistent with the calibration project's decision
  D-10 as revised (the second sphere was already a linearity check rather than a requirement).

## Consequences for the calibration project's documents

- Procedure: Section 1b (drop sphere A and its mounts; costs), 1c (micrometer range: only the 150 to 175 mm size is
  needed), 1d (one sphere column), 1e unchanged, 3a (drop the steel-sphere bonding items, keep the aluminum sphere
  B items), 3b (one tool center point), Appendix A (suppliers of sphere A), Appendix D (retire SC1-01 and SC1-03).
- Cost script and both decks: fixtures, build list, buy list, cost, sphere specification, suppliers, mounting and
  sphere-end slides, and every speaker note that counts two spheres.
- Analysis decision log: a new entry recording Decision 1 and 2 with the reasoning above, and D-10 marked revised.
- The scaling check is gone from the plan; say so once where the two-radius overlap was described.

## Related decisions in the performance project that the calibration project should know

- The 400 x 400 mm noise plate was eliminated; the performance project's T2 is now the stage-1 board (200 x 150 mm,
  on its SC1-05 adapter), so one board serves all three projects, and the performance project's registration is the
  plane-correspondence method of the registration project.
- No feature-level measurement of targets is made in-house. The fabricator's inspection report fills the as-built
  record; in-house checks are limited to a sphere's diameter with a micrometer, a plate's flatness with the run-out
  gauge, and a plate's width and length. The drawings therefore carry tolerances.
- The robot model is undecided; every document stays agnostic of it, with the adapter drawn to ISO 9409-1-50-4-M6
  and a boxed "confirm against the robot's flange drawing" note, as in the stage-1 drawings.
- The feature targets of the performance project mount by a spigot on the back of each target into a bore of a
  new adapter, held by a side ball-lock pin, a variant of the stage-1 stem concept applied to plates.

## Open items for the calibration discussion

1. Which distance range the sweep uses (the calibration volume of 300 to 1100 mm, or the wider 400 to 1600 mm).
2. Whether SC1-01 and SC1-03 are retired or kept as archived drawings.
3. The updated cost figures and the single-sphere purchase specification.
