# Z-sweep redesign of the disk, cutout, station and registration design

Decided during the specification review (chunk C03, 2026-10-05). This note is the
single statement of the change for the specification, the code and the technician
procedure. Values at the indicative geometry (f_x = 688.155 px, 640 x 480, baseline
75 mm) are examples; the rules are the definition.

## 1. Working range and station ladder

- `Z_MIN_MM` = 400, `Z_MAX_MM` = 1600 (both marked dagger: Step 4.5 confirms the
  sensor reads at both ends). The robot distance may vary over this range for every
  series.
- One geometric station ladder for the whole procedure:
  `Z_STATION_RATIO` = 2^(1/4) (four stations per octave), stations
  Z_MIN * ratio^k rounded to 1 mm: 400, 476, 566, 673, 800, 951, 1131, 1345, 1600
  (`station_count` = 9). This replaces `Z_NOISE_STEP_MM` (A), `Z_SHAPE_STATIONS_MM`
  and `Z_REDUCED_STATIONS_MM` as lists.
- Subsets by stride: `Z_SHAPE_STATION_STRIDE` = 2 gives the B-HV stations
  400, 566, 800, 1131, 1600 (5); `Z_REDUCED_STATION_STRIDE` = 4 gives the B-Z and
  tilt-sub-series stations 400, 800, 1600 (3).
- A (noise), C (area) and D (detection) use all 9 stations. A also adds the two
  legacy depths `LEGACY_METRIC_DEPTHS_MM` = 700, 1000 as extra stations so the
  legacy metrics are computed at the same depths as the existing data.
- `Z_REFERENCE_MM` = 800 (a station): warm-up check, sentinels, re-mount check, the
  C field sub-series, the open-background variant and the D post check.
- Registration poses span Z_MIN to Z_MAX.

## 2. Feature ladder (disks and cutouts)

- Subtended size in pixels D_px = D * f_x / Z is the governing variable. Over the
  Z range (factor 4, two octaves) one feature covers two octaves of D_px.
- `FEATURE_LADDER_RATIO` = 2 * sqrt(2) (half-octave overlap between neighbors),
  `FEATURE_MIN_PX_AT_Z_MAX` = 3 (smallest feature in pixels at the far station),
  `FEATURE_COUNT` = 3. Diameters D_k = FEATURE_MIN_PX_AT_Z_MAX * p(Z_MAX) * ratio^k:
  7.0, 19.7, 55.8 mm at the indicative geometry, covering 3.0-12, 8.5-34 and
  24-96 px.
- Rationale (person's model of the VSX3000): about 300,000 depth pixels and about
  30,000 projected laser pencils (10 px per pencil); the path-correlation needs at
  least 4 pencil detections, about 40 px of area, a diameter of about 7 px; with the
  support window of the matcher (7 x 7 to 13 x 13 px in documented active-stereo
  ASICs) a minimum detectable diameter of 10-15 px is plausible. The ladder brackets
  it from 3 px to 96 px.
- One disk plate T4 and one cutout plate T5 replace T4-S/T4-L and T5-S/T5-L.
  Each carries `FEATURE_COUNT` features, `BLANK_SITES_PER_PLATE` = 3 blank sites
  sized to the largest search window, and (disks only) `POST_SITES_PER_PLATE` = 1
  post-only site. `FEATURE_ISOLATION_PX` = 30 is now evaluated at Z_MAX
  (30 * p(Z_MAX) = 70 mm) so neighbors stay separated at the far station.
- The pixel-footprint ladder rules `DIAMETER_LADDER_RATIO`,
  `DIAMETER_MIN_FOOTPRINT_FRACTION`, `DIAMETER_MAX_FOOTPRINT_MULTIPLE` and the
  detection level rules `DETECTION_LEVELS`, `DETECTION_LEVEL_LOW_FACTOR`,
  `DETECTION_LEVEL_HIGH_FACTOR`, `DETECTION_FINE_LADDER_RATIO` are removed.
  Detection levels are the (feature, station) pairs: 3 x 9 = 27 values of D_px
  spaced by 2^(1/4) with the overlaps.
- `DETECTION_TRIALS_PER_LEVEL` = 60 trials per (feature, station).
  `DETECTION_ZERO_TRIALS` = 300 at the `DETECTION_ZERO_STATION_COUNT` = 3 farthest
  stations (1131, 1345, 1600), where the smallest feature lies below the expected
  threshold. The D pilot keeps only the post check (post-only sites not detected);
  level selection is gone. The former optional continuous-angle variant is the
  main design and is removed as a variant.

## 3. Registration without a pattern

- T1 (patterned registration plate) is removed. Registration uses the noise plate
  T2 by plane correspondence (the plane-only hand-eye solve already in
  `sensorperf/geometry/registration.py`): camera_to_base is fully observable; the
  in-plane position of the plate on the flange and its rotation about its normal
  are not, and are not needed for T2.
- In-plane datum for targets with features (T3a, T3b, T4, T5): once per mount,
  the front plane is fitted from depth (Z and tilt) and one feature edge or outline
  is located in the left IR image (H and V), independent of the depth pipeline;
  agreement with the as-built record within `FRAME_CHECK_PX` is required. Every
  target carries the same dowel datum and the as-built record includes the
  datum-to-feature offsets (cross-check).

## 4. Equipment changes (Section 3.1)

- The "Enclosure or blackout curtains, IR light meter" row is removed (the
  laboratory is enclosed with constant lighting); the ambient-IR manifest column
  and mentions go with it.
- The robot must report the actual (encoder-derived) pose at
  `POSE_LOG_RESOLUTION_MM`, time-stamped against the frames; repeatability
  `ROBOT_REPEATABILITY_MM`. New named constants: `ADAPTER_REMOUNT_REPEATABILITY_MM`
  = 0.02, `TEMPERATURE_LOG_INTERVAL_MIN` = 1.

## 5. Analysis changes

- C (area): transfer curves A_sensed/A_true and A_sensed/A_geo against D_px pooled
  over features and stations, feature identity kept as a factor. New scaling test:
  where two features overlap in D_px, compare their curves; agreement within the
  bootstrap interval supports D_px as the governing variable, disagreement is
  attributed to sigma_tot(Z) and reported.
- D (detection): psychometric fit on ln D_px pooled over (feature, station) pairs;
  gamma from the blank sites per station; the same overlap test; D_0 reported as a
  D_px bracket and converted to mm at each station. The Z-dependent noise enters
  as a covariate (logistic regression on ln D_px and ln sigma_tot(Z)).
- E unchanged except the feature-scale profiles are against D_px.

## 6. Budget

Recomputed by the plan tool (`sensorperf/acquisition/plan.py`) from the parameters;
Section 9 of the specification carries the tool's numbers.
