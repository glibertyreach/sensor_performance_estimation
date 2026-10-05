# VSX3000 sensor performance testing: step-by-step procedure

Audience: the robot technician and the test engineer. The technician mounts the targets, programs the robot, and records the captures. No knowledge of the analysis math is needed. Where a step says "run", a computer with Python and this repository is needed (appendix C says how to set it up). The steps that need the engineer are marked "Engineer": filling in the sensor values from the SDK or datasheet (§3), the hand-eye solve and its acceptance (§4), choosing the sensor configuration and any filters-off repeat (§3), the pilot and level choice for series D (§10), the as-built measurements of the targets (§2), and running the analyses (§14). In this document, "§" followed by a number means that section of this document.

What you are producing: a session folder of sensor capture files (`.mc`), one group of files per robot pose, with a table (the manifest) that says, for every file, which target was in view, where the robot had put it, and what the environment was. The specification calls this the data logging of its Section 9. The analysis software reads the manifest, not the file names. If the manifest is wrong, the results are wrong, so much of this procedure is about getting the table right.

The analysis definitions (what is computed from the captures) are in the characterization procedure specification of 2026-10-04. This document is the step-by-step rendering of its Part I (acquisition) plus a guide to running the analyses of its Part II. It does not repeat the math. "Specification Section N" means that section of the specification.

The order of the five capture series, and what each analysis needs from the others, is in Figure 1. Figure 2 (§1) shows the setup. Figure 3 and Figure 4 (§2) show the targets and their chamfered edges. Figure 5 and Figure 6 (§5) show the stations and the planned poses. Figure 7 (§8) shows the depth-step visits, and Figure 8 (§13) shows how the session folder feeds the analyses.

![](figures/fig_procedure_flow.png)

Figure 1. Procedure order and data dependencies. Shaded boxes are captures and open boxes are analyses. Registration poses feed every analysis, the noise level from A sets the thresholds of B, C and D, the edge spread function from B predicts the area bias measured in C, and E reuses the B and C frames. The quick-look count on C data sets the D levels (dashed).

---

## 1. Equipment

The sensor stays fixed. The robot carries the target, so every target pose is commanded, repeatable, and logged. Figure 2 shows the arrangement. Table 1 lists the equipment and why each item is needed.

![](figures/fig_setup.png)

Figure 2. The experimental setup, side view. The sensor stands on its own rigid stand, separate from the robot. The robot carries the target on the dowel-pinned adapter anywhere between Z_MIN and Z_MAX. The dial indicator on its fixed stand reads the adapter's Z move in series Z. The enclosure holds the ambient IR constant.

| Item | Requirement | Why |
|---|---|---|
| VSX3000 sensor on a rigid stand | Stand mechanically separate from the robot base, on an isolated floor or table | Prevents robot motion from moving the sensor |
| 6-axis robot | Repeatability 0.05 mm or better (ISO 9283); payload above the heaviest target plus adapter | Commanded, repeatable target poses |
| Quick-change target adapter | Dowel-pinned, re-mount repeatability 0.02 mm or better | Targets can be swapped without re-registering |
| Dial indicator or capacitive probe on a fixed stand | Resolution 1 micrometer or better | Verifies the smallest Z steps (series Z), which are near the robot's repeatability |
| Enclosure or blackout curtains, IR light meter | Ambient IR held constant and logged | Ambient IR changes the noise level |
| Temperature loggers (sensor housing and air) | 1 sample per minute | For drift attribution |
| Capture computer | VSX3000 SDK, the LRVisionLibs `MatCloud` reader, and a robot interface that logs the actual pose read back from the robot | Synchronized capture and logging |

Table 1. Equipment, requirements, and the reason for each.

The robot must report its actual pose at the time of the capture, not only the commanded one. If it cannot, tell the engineer before you start (appendix E, open question 5).

## 2. Targets

Every target uses a two-plane construction: a front surface with knife edges, standing a known gap G in front of a back plate of the same finish. One geometry therefore serves the edge, area, detection, and boundary-bias tests. The two gaps are 15 mm (the small gap) and 60 mm (the large gap), set with spacers. Fabrication tolerances are left to the fabricator. Instead, every feature is measured as built and logged (the as-built record, below). Table 2 lists the targets and Figure 3 shows them to one scale.

| ID | Target | Construction | Used in |
|---|---|---|---|
| T1 | Registration plate | Flat, about 400 by 300 mm, with a ChArUco or circle-grid pattern visible in the left IR image. The pattern geometry is known to 0.02 mm or better. | Registration (§4) |
| T2 | Noise plate | Uniform matte, 400, 400 mm (width, height), flat to 0.05 mm | A, Z, sentinels |
| T3a | Raised square | Square of side 160 mm with knife edges, on hidden posts at the gap G above a back plate. Spacers set G to 15 or 60 mm. | B edges, E |
| T3b | Square window | Front plate with a square window of the same size and knife edges. The back plate is at G behind it. | B edges, E |
| T4-S, T4-L | Disk arrays, small and large | Back-beveled disks on thin posts at G above a back plate. The diameter ladder is split across two plates by size. Includes blank sites and post-only control sites. | C, D, E |
| T5-S, T5-L | Cutout arrays, small and large | Back-beveled holes in a front plate, with the back plate at G. Includes blank sites. | C, D, E |

Table 2. The targets (specification Section 3.2).

![](figures/fig_targets.png)

Figure 3. Front views of T2, T3a, T3b, T4-S, T4-L, T5-S and T5-L to one scale, drawn from the code's own target definitions with the indicative sensor geometry. The real layout depends on the final focal length (unconfirmed). Gray is the back plate, dashed circles are blank sites, vermillion dots are post-only control sites.

**Diameter ladder.** Diameters run from 0.3 times the pixel footprint at Z_MIN up to 30 times the pixel footprint at Z_MAX, in steps of a factor 1.41421. With the indicative f_x of 688 px (unconfirmed) this gives 0.22 mm to 55.8 mm, 17 diameters in all. The small arrays carry the lower half of the ladder and the large arrays the upper half. Features on a plate are spaced at least 30 px apart at Z_MIN (edge to edge), so the sensor's spatial interpolation cannot couple neighboring features.

**Two planes.** In the disk arrays and T3a the front material is the disks (or the square) and the back plate is seen around them. In the cutout arrays and T3b the front material is a plate and the back plate is seen through the holes (or the window). Spacers set the gap. Measure the real gap and record it.

**Blank and control sites.** Each array has blank sites matching each search-window size. These give the false-alarm rate in series D. Disk arrays also have post-only sites (a post with no disk). These show whether the support post itself is detected.

**Surface finish.** All front and back surfaces use one finish, for example bead-blasted aluminum or a matte coating, with mid-range IR reflectance. Record the finish and, if possible, its reflectance at the projector wavelength. A reflectance difference between plates would bias the edge and area results.

### Chamfered (knife-edge) boundaries

Every boundary that defines an edge, disk, or cutout must be chamfered from the back (Figure 4). Then no camera or projector ray can strike the boundary's side wall, and the sensor sees only the front face and the back plate. An unchamfered wall would be seen as a third surface by one camera and not the other, which corrupts exactly the edge measurements this procedure is meant to make.

![](figures/fig_chamfer.png)

Figure 4. Chamfered cutout and disk on its post, cross-section, not to scale. The three viewpoints (left camera, projector, right camera) pass the knife edge in open space and never meet the beveled wall. The bevel angle is measured from the plate normal.

- Cutouts and the square window: countersink from the back face, so the hole widens away from the sensor.
- Disks and the raised square: bevel the back, so the part narrows away from the sensor (a frustum).
- Land: the flat land left at the front edge must be 0.2 mm or less.
- Bevel angle: measured from the plate normal, it must exceed the worst-case ray angle plus a margin. The margin is 10 degrees. The worst-case ray angle is the largest angle between the plate normal and the line from any of the three viewpoints (left camera, right camera, projector) to any edge point, over every pose used.

A rough worked value from the specification, assuming about a 70 degree horizontal field of view and a 50 mm baseline (both unconfirmed): the worst field position at Z_MIN gives about 32 degrees. With the margin that is 42 degrees, so a standard 45 degree bevel (90 degree included countersink) would pass. The engineer recomputes this once the VSX3000 geometry is known. If the result exceeds 45 degrees, use a steeper bevel or reduce the field offset (0.6 of the half field) for series C, D, and E.

**Disk support posts.** The post must sit behind the disk and be thinner than 0.5 times the pilot D_0 (§10). For the smallest disks this means posts well under 1 mm, such as hypodermic tubing or wire. The post-only control sites confirm the post is not detected. An alternative for the smallest disks is suspension on taut sub-resolution wires, also with wire-only control sites.

### The as-built record

Engineer, with the metrologist: measure each feature's front-face diameter, land width, bevel angle, and plate position with an optical comparator or a calibrated microscope. Record each value with its measurement uncertainty in `targets_asbuilt.csv`. This is the file the analysis reads. All analyses use these as-built values, never the nominal ones. Table 3 lists the columns. The engineer can write a template with the nominal values for the metrologist to overwrite; an empty numeric cell keeps the nominal value.

| Column | Meaning |
|---|---|
| `target_id` | Target (T3a, T4-S, and so on) |
| `site_id` | Name of the site on the target, as in `targets.json` (for example `disk_03`, `blank_03`, `post_00`, `square`) |
| `kind` | Disk, cutout, blank, post, raised square, or square window |
| `x_mm`, `y_mm` | Position of the site center on the front face, from the plate center, in mm |
| `diameter_mm` | Front-face diameter (the side, for a square), in mm |
| `diameter_uncertainty_mm` | Measurement uncertainty of the diameter, in mm |
| `land_mm` | Width of the flat land at the front edge, in mm |
| `bevel_deg` | Bevel angle from the plate normal, in degrees |
| `rotation_deg` | In-plane rotation of a square feature (the slant), in degrees |
| `level_index` | Rung of the diameter ladder (disks, cutouts, blanks); empty otherwise |

Table 3. Columns of `targets_asbuilt.csv`.

## 3. Before anything else

Do these steps in order, once, before the first capture of any series. They fix the conditions of the whole session.

1. Environment. Close the enclosure and set the lighting to its fixed state. Log ambient IR and air temperature in `environment_log.csv`. Keep these unchanged for the whole session. Start the temperature loggers.
2. Sensor configuration (Engineer). Disable auto-exposure and fix exposure, gain, emitter power, and trigger mode. Record every depth-processing setting (temporal filter, spatial filter, hole filling, confidence threshold) in `sensor_config.json`, with its SDK name and value. Characterize the configuration that production will use. If the filters can be switched off, the engineer decides whether to run series A a second time with the filters off, so the sensor's own processing can be separated from its physics (§6, step 8). Give the configuration a short identifier (`config_id`); it goes into every manifest row.
3. Warm-up. Power the sensor for at least 45 minutes. Then put T2 fronto-parallel at Z = 750 mm and capture 10 frames every 1 minute. The engineer computes the mean plane Z of each capture. Start testing once the mean plane Z has drifted less than 0.1 times sigma_t (the frame-to-frame depth noise at that Z) over 10 minutes.
4. Settle and vibration check. With T2 at Z_MAX (1000 mm), capture 100 frames twice: once with the servos on, after a move and the settle wait, and once with the brakes engaged. If the servo-on sigma_t exceeds the brakes-on sigma_t by more than 0.1 times the brakes-on value, raise the settle time (now 2 s) or stiffen the target mount, then repeat. Whatever settle time passes this check is the one the robot program uses in §5.
5. Intrinsics and frame checks (Engineer).
    - Read the left IR intrinsics, the depth-to-IR extrinsics, the stereo baseline, the depth LSB, and the projector offset from the SDK or datasheet. These are the values marked with a dagger in appendix A. Write them into `sensor_config.json` under `geometry` (the focal lengths and principal point in pixels, the image size, the baseline and projector offset in mm, the depth LSB in mm, and the frame rate). Until this is done the planner and the code use indicative values.
    - Confirm the depth image is registered to the left IR image. With T3a in view, overlay the square's edges from the IR image on the depth discontinuities. They must agree within 0.5 px.
    - Confirm the baseline direction. The occlusion band (a strip of no-reads) appears beside vertical edges only if the baseline runs along H. If it appears beside horizontal edges, tell the engineer: H and V swap in series B and E.

Example of `sensor_config.json` (the text in angle brackets is replaced by the real value; the keys are the ones the software reads):

```
{
  "config_id": "<short id>",
  "serial_number": "<sensor serial number>",
  "exposure": <value>, "gain": <value>, "emitter_power": <value>,
  "trigger_mode": "<name>",
  "filters": {"temporal": <setting>, "spatial": <setting>,
              "hole_filling": <setting>, "confidence_threshold": <setting>},
  "geometry": {"sensor_fx_px": <>, "sensor_fy_px": <>, "sensor_cx_px": <>, "sensor_cy_px": <>,
               "image_width_px": <>, "image_height_px": <>, "sensor_baseline_mm": <>,
               "projector_offset_mm": [<H>, <V>, <Z>], "depth_lsb_mm": <>, "frame_rate_hz": <>},
  "notes": ""
}
```

Write the date, sensor serial number, firmware and SDK versions, robot model and controller software version, the active base frame name, the plate flatness report, the finish, and anything unusual in `session_log.md`. Do not change the active base frame, the exposure, or any sensor setting during the session.

## 4. Robot-to-sensor registration

Registration gives every later capture a ground-truth target pose in the sensor frame. Two things anchor it. Robot repeatability fixes relative motion. An IR-image fiducial solve fixes absolute pose, and that solve does not use the stereo depth pipeline. After step 5 below, the robot-base-to-sensor transform is known, so every commanded target pose is also a ground-truth pose in sensor coordinates (compare Figure 2).

1. Mount T1 on the dowel-pinned adapter. Do the mount check of step 7 once now.
2. Take the registration rows of the plan: 30 poses that span Z_MIN to Z_MAX, cover the field of view, and tilt within plus or minus 20 degrees about H and V. In the plan they have procedure letter `R`. Registration comes first, so the robot cannot yet be commanded in sensor coordinates: jog the robot by hand to each pose, using the live IR image to reach about the planned depth, field position, and tilt (T1 must be fully visible). The exact pose is solved afterward, so hand-jogged poses are fine.
3. At each pose, capture 10 frames: the IR image, the depth image, and the read-back robot pose. Name the files as in §5.
    - If the SDK can switch off the emitter or switch on a flood illuminator, capture the IR fiducial frames that way. The projected dot pattern degrades corner detection.
    - If neither is possible, register from depth-plane fits alone. In that case a constant depth offset cannot be told apart from registration translation (§15).
4. Engineer: solve the hand-eye problem for the camera-to-robot-base transform and the plate-to-flange transform. The fiducial solve gives the target pose in the camera frame at each pose. Write one row per pose into an observations file: `pose_id`, the read-back flange pose (`x_mm`, `y_mm`, `z_mm`, `rotation_type`, `r1` to `r9`, as in §11), and either the target pose (`tx_mm`, `ty_mm`, `tz_mm`, `trx_deg`, `try_deg`, `trz_deg`, a rotation vector in degrees) or, for a depth-plane registration, the plane (`nx`, `ny`, `nz`, `distance_mm`). Then run:

```
python3 -m sensorperf.cli.register --observations observations.csv --out registration.json
```

   Add `--method planes` for the depth-plane form. The tool prints the residual and the verdict.
5. Accept the registration only if the RMS distance between the solved plate poses and the measured plate poses is 0.15 mm or less. If it is more, the tool still writes `registration.json` but marks it not accepted, and its exit code is 1. Re-capture the worst poses or check the mount, then solve again. Keep the residual with every result.
6. Save both transforms in `registration.json` in the session folder.
7. Once per mount of any target: fit the mounted target's front plane at Z = 750 mm and compare the fit with the registered pose. This checks that re-mounting on the dowel-pinned adapter really needs no new registration. Log the result in `session_log.md`.
8. Station targets. For every row of the plan, the robot pose that puts the target's reference point at the required (H, V, Z) in the camera frame, fronto-parallel unless the row says otherwise, comes from the registration. The planner computes it (§5).

## 5. The plan

Run the planner once the registration is accepted and the sensor values are in `sensor_config.json`. It writes the commanded pose of every capture, in the order to do them.

```
python3 -m sensorperf.cli.plan_stations --out plan/ --registration registration.json \
    --sensor-config sensor_config.json --seed 1
```

Write the seed in `session_log.md`. The same seed and inputs give the same plan again. Without `--registration` the planner writes the target poses in the camera frame only; without `--sensor-config` it uses the indicative sensor geometry and says so loudly, because the field-of-view fit, the lateral offsets in millimeters, the expected depth quantum, and the capture budget (§13) all depend on the real values. Appendix B lists every option.

The planner writes five files in the output folder:

- `poses.csv`: one row per commanded pose.
- `plan_summary.txt`: poses and frames per series and station, the capture budget, every adjustment, and every warning. Read it before you start.
- `plan.png`: the planned target centers, a picture of the same kind as Figure 6.
- `targets.json`: the target definitions the plan was made with.
- `parameters.json`: the parameter values the plan was made with (appendix A).

The stations the plan visits, and the five field positions in the image, are in Figure 5. The whole plan is in Figure 6.

![](figures/fig_stations.png)

Figure 5. Left: the 11 noise stations of A, the 5 shape stations of B, C and D, and the 3 reduced stations of the slow tests, along Z. Right: the 5 field positions (codes 0 to 4) in the image, and the square span of the random lateral offsets (phase jitter) at one corner, true size and magnified.

![](figures/fig_plan.png)

Figure 6. The default full plan: target centers in the sensor frame, side view (H against Z) and front view (H against V), colored by series, with the frustum. Each cloud around a station is a set of random lateral offsets.

**What a row of `poses.csv` contains.** Table 4 lists the procedure letters. A row has the order to execute it in (`order`), the identity that names the files (`procedure`, `target_id`, `gap_mm`, `station_z_mm`, `field`, `pose_index`), the number of frames (`frames`), a sub-series label (`subseries`: for example `main`, `tilt`, `remount`, `nominal`, `jitter`, `ladder`, `staircase`, `field`, `extended`, `sentinel`), the logged random seed (`seed`), the random lateral offset in mm at the station depth (`offset_h_mm`, `offset_v_mm`), the tilt (`tilt_axis`, `tilt_deg`), the commanded Z step and visit (`step_mm`, `visit`, series Z only), the diameter level (`level_index`), and the wanted target pose in the camera frame (`target_x_mm` to `target_rz_deg`: position in mm and a rotation vector in degrees). With a registration the row also holds the flange pose to command in the robot base frame: position and rotation vector (`base_x_mm` to `base_rz_deg`), the rotation matrix (`r00` to `r22`, row by row), and the quaternion (`quat_w`, `quat_x`, `quat_y`, `quat_z`). Use whichever form your robot program accepts. A last column (`notes`) holds extra detail as JSON.

| Letter | Series | Target | See |
|---|---|---|---|
| R | Registration | T1 | §4 |
| A | Noise plate | T2 | §6 |
| B | Edges | T3a, T3b | §7 |
| Z | Depth steps (B-Z) | T2 | §8 |
| C | Disk and cutout areas | T4-S, T4-L, T5-S, T5-L | §9 |
| D | Detection trials | T4-S, T4-L, T5-S, T5-L | §10 |
| S | Drift sentinels | T2 | §6, step 3 |

Table 4. Procedure letters of the plan and of the file names.

The planner checks that each target fits the field of view at its station. A target that would not fit, for example T2 off-axis at Z_MIN, is pulled inward along its field direction until it fits. `plan_summary.txt` lists every such adjustment and every target that does not fit even when centered. Do not override these. Tell the engineer about any plate that does not fit when centered.

**Robot program outline**, for each row of `poses.csv`, in `order`:

1. Move to the pose. Use a joint move to a point short of it along the target normal, then a linear move onto it, so the approach is the same every time.
2. Wait the settle time of §3, step 4 (2 s unless the check raised it).
3. Trigger the capture of `frames` frames. Name the files `<proc>_<target>_G<gap>_Z<zzzz>_F<field>_P<pose>_f<frame>.mc` from the row: for example `C_T5S_G15_Z0750_F0_P017_f03.mc` is procedure C (area), target T5-S (the hyphen is dropped), gap 15 mm, station Z = 750 mm, field position 0 (center), pose 17, frame 3. A target without a back plate (T1, T2) writes `G0`. The frame number runs from `f00`.
4. Read the robot's actual reported flange pose (not the commanded one) and append it to the pose log (§11). Add the dial-indicator reading at every visit of series Z, and the sensor and air temperature, the ambient IR, and a timestamp.
5. Move on.

Sentinel rows (letter `S`) appear in the plan every 60 minutes, by the planner's estimate of the clock. A sentinel is T2 at the center at Z = 750 mm, 30 frames. It needs T2 on the robot. If T2 is not mounted when a sentinel row comes up, ask the engineer whether to swap targets now or to do the sentinel at the next target change, and write the actual time of the capture in the pose log. After the sentinel, mount the target of the series again and do the mount check of §4, step 7.

Do the series in the order of the plan: A, then B, then Z, then C, then D. Do not move the sensor between them. Tell the engineer at once if a series stops early; the later series depend on the earlier ones (Figure 1).

## 6. Series A: the noise plate

This series captures 11 noise stations at 5 field positions each, 100 frames per pose, plus a tilt sub-series. The station order is randomized, with a logged seed, so that slow drift cannot masquerade as a Z dependence. The planner has already done this: follow the `order` column.

1. Mount T2. Do the mount check (§4, step 7). The noise stations are every Z from Z_MIN (500 mm) to Z_MAX (1000 mm) in steps of 50 mm. The five field positions are the center and four corners at 0.6 of the half field, all fronto-parallel (Figure 5).
2. Check that the plate fully covers the analysis region at every station. At Z_MIN off-axis this may limit the field offset. The planner has pulled such poses inward; see `plan_summary.txt`.
3. Before the first station, capture a drift sentinel: center, Z = 750 mm, 30 frames. The plan repeats the sentinel every 60 minutes and after the last station.
4. At each station: move, wait the settle time, then capture the frames of the row. Log the read-back robot pose, the sensor temperature, and the timestamps.
5. Tilt sub-series. At the center and at the reduced stations, T2 is tilted about V, then about H, through each angle of 0, 15, 30, 45 degrees. Capture 50 frames per pose. Incidence angle changes both the per-pixel noise and the fill rate. These rows have sub-series `tilt`.
6. Repeat-mount check. Dismount T2, re-mount it, and repeat the Z = 750 mm center station (sub-series `remount`). The difference shows how much of the bias comes from re-mounting the target.
7. Do not delete any frames. Tell the engineer if a pose shows a warning in §12.
8. Engineer: if step 2 of §3 calls for it, repeat the series with the sensor's filters off. Plan it with `--filters-off` and record the new configuration in `sensor_config.json` with a new `config_id`.

## 7. Series B: the edge targets

The edge series gives lateral (H, V) resolution and most of the boundary-bias data. Both edge polarities are measured: T3a (front material inside the square) and T3b (back plate inside the window). At each station the target is moved by small random lateral offsets so that every sub-pixel phase of each edge is sampled.

1. Mount T3a (raised square) with the gap at 15 mm. Mount it square to the image: the slant of 5 degrees is part of the square (the as-built record gives it as `rotation_deg`), so all four edges are slanted relative to the pixel grid. The two near-vertical edges measure H resolution and the two near-horizontal edges measure V. Left and right edges have opposite occlusion geometry relative to the baseline, and so do top and bottom. Do the mount check (§4, step 7).
2. At each shape station (500, 625, 750, 875, 1000 mm), centered and fronto-parallel: move, settle, and capture 30 frames at the nominal pose (sub-series `nominal`).
3. At the same Z, capture 25 further poses, each with 30 frames (sub-series `jitter`). Each adds a logged random lateral offset, uniform over plus or minus half of the phase-jitter span (8 px at the station Z) in both H and V. The plan holds the offsets in millimeters (`offset_h_mm`, `offset_v_mm`), so the robot only has to execute the row.
4. Repeat steps 2 and 3 with the gap at 60 mm. Comparing the two step heights tests whether the normalized edge response depends on step height. If it does, the depth pipeline is nonlinear and resolution must be quoted together with its step height.
5. Repeat steps 1 to 4 with T3b (square window). The window has the opposite edge polarity: front plate outside, back plate inside.

Follow the order of the plan, which keeps each target mounted for as long as it can.

## 8. Series Z: depth steps

The depth series gives the smallest Z step the sensor detects. It uses small Z moves of the noise plate, checked by the dial indicator. This is the series most limited by the robot's repeatability, which is why the indicator is the truth.

![](figures/fig_zstep.png)

Figure 7. The series Z visits at one Z0. Left: the step ladder, the 8 step sizes in turn, each with its A, B, A, B alternation. Right: the fine staircase. A circle marks the dial-indicator reading logged at every visit.

1. Mount T2 centered and fronto-parallel at each Z0 in the reduced stations (500, 750, 1000 mm). Put the dial indicator against the back of the target adapter, along the target normal (Figure 2). Zero it at Z0.
2. Step ladder. For each of the 8 step sizes (0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 4 mm), alternate the target between Z0 (visit A) and Z0 plus the step (visit B) for 10 cycles (A, B, A, B, and so on). Capture 10 frames at each visit. Log the dial-indicator reading at every visit: the analysis uses the indicator value, not the commanded step, as ground truth for the step. Alternating cancels linear drift. Rows have sub-series `ladder`, with `step_mm` and `visit` filled in.
3. Fine staircase. Sweep from Z0 to Z0 plus 3 times the expected depth quantum, in steps of one expected quantum divided by 10, with 10 frames per step and the indicator logged at each (sub-series `staircase`). This reveals whether the output moves in quantized steps, or is smoothed by the sensor's interpolation. The expected quantum, listed in `plan_summary.txt`, uses the Tier-A disparity quantum until analysis A has measured the real one. If the engineer has re-planned the staircase with the measured quantum, use the new rows.
4. No extra capture is needed for the blank windows: the Z0 frames of step 2 serve as the no-step reference for the false-alarm threshold.

The smallest steps (the first rungs of the ladder) are below the robot's repeatability. Do not try to correct the robot pose by hand. Log the indicator reading as it is.

## 9. Series C: disk and cutout areas

Each array is captured at many random sub-pixel offsets. Sensed area depends on where a feature's edge falls relative to the pixel grid and the projector dots. The procedure averages over that phase and also measures its spread.

1. The configuration list is every array of {T4-S, T4-L, T5-S, T5-L}, with both gaps, at every shape station. Center the array and keep it fronto-parallel. The plan randomizes the order within each mounting, so targets are re-mounted as rarely as possible. Mount one array at a time and do the mount check (§4, step 7).
2. At each configuration, capture 30 poses, each with 10 frames (sub-series `jitter`). Each has a logged random lateral offset, uniform over plus or minus half of the phase-jitter span in H and V.
3. Field sub-series. At Z = 750 mm and the small gap, repeat step 2 for each array at the four corners, 10 poses each (sub-series `field`).
4. Open-background variant (cutouts, optional). At Z = 750 mm, remove the back plate so that nothing lies within the sensor's range behind the holes. Then repeat step 2. This separates the sensor's fill-in behavior from reads of a real back surface. Plan it with `--open-background`.
5. Capture the drift sentinels the plan lists (§5).

Note which configuration the pilot of §10 uses: its first frame of each C pose at Z = 750 mm.

## 10. Series D: detection trials

A detection trial is one frame at a fresh random lateral offset. All features and blank sites sit on one plate, so a single frame yields one trial for every level on that plate at once. The 10 percent and 0 percent minimums need two things the 50 percent point does not: levels that reach down to where detection disappears, and many more trials at those levels.

1. Pilot (Engineer). Run the quick-look detection count on the first frame of each C pose at Z = 750 mm:

```
python3 -m sensorperf.cli.check_captures --session Characterization_20261014/ --pilot 750
```

   From the counts the tool estimates a pilot D_50 and a pilot D_0 for each array and gap. If any post-only control site shows detections above the false-alarm rate, the disk posts must be re-made thinner and the pilot repeated. Pass the pilot values to the planner with `--pilot-d50-mm` and `--pilot-d0-mm` (appendix B).
2. Level selection (Engineer). The levels are 9 sizes spanning 0.3 to 2.5 times the pilot D_50. Check that at least two levels fall below the pilot D_0. The 0 percent estimate needs levels with no detections, and the 10 percent estimate needs points on the lower tail. If the ladder is too coarse to give that many levels, a dedicated detection plate is made, built like T4 and T5 with the same blank and control sites, whose diameters differ by a factor of 1.18921. The technician mounts whichever plates the engineer names.
3. Main trials. For each configuration (disk or cutout, either gap, any shape station), capture 60 poses, each with a new random lateral offset and 1 frame. The plan shuffles the pose order with a logged seed. Follow the `order` column. The plan treats each plate separately, because a frame sees only one plate.
4. Extended trials for the 0 percent point. At each reduced station, the plan raises the pose count of every configuration to 300 (sub-series `extended`). Every level, and every blank site, then has that many trials. With zero detections in n trials, the one-sided upper bound on detection probability at 0.95 confidence is about 3/n. So 300 trials bound a "never detected" level at 1.0 percent (0.01). If time is short, the engineer can lower the number of extended trials: for example 150 trials bound the 0 percent point at 2 percent and roughly halve that part of the budget.
5. Independence rule. Never use two frames from the same pose as separate trials. The fixed projector pattern and the target's fixed position make them strongly correlated. Take one frame per pose, always at a new offset. The analysis checks independence after the fact.
6. Reuse. The first frame of each C pose at a matching configuration may count as a D trial. The plan does not count it, so it does not reduce the number of D poses planned.
7. Optional continuous-angle variant (Engineer). Fix a single size near the pilot D_50. Sweep Z from Z_MIN to Z_MAX in 10 mm steps, with 20 random-offset poses per step. This samples subtended angle continuously, but it mixes the angle effect with the noise change over Z.

## 11. Recording the poses: the pose log and the manifest

The capture software writes the sensor data. The robot pose must be recorded separately. The manifest records the read-back pose, not only the commanded one. The robot program appends one line per pose, or one per frame, to a CSV file, the pose log. Report the flange pose in the robot base frame, in the same frame for the whole session. Do not change the active base frame or tool frame.

Per-pose columns (one row for the frames 0 to `frames` minus 1 of a pose):

```
procedure, target_id, gap_mm, station_z_mm, field, pose_index, frames,
x_mm, y_mm, z_mm, rotation_type, r1, r2, r3, r4, r5, r6, r7, r8, r9
```

Optional columns that end up in the manifest: `timestamp`, `indicator_mm` (the dial indicator, series Z), `sensor_temp_c`, `air_temp_c`, `ambient_ir`. A one-row-per-frame form also exists: the first column is `file`, the Section 9 file name, followed by `x_mm` and the rest.

- `procedure` to `pose_index`: exactly the values of the plan row, which also name the files. `gap_mm` is empty for a target without a back plate.
- `x_mm`, `y_mm`, `z_mm`: the reported flange position in the robot base frame, in mm.
- `rotation_type` and `r1` to `r9`: the reported orientation, in whatever form the controller gives, named by one of the types of Table 5. Fill unused `r` columns with nothing.

| rotation_type | Values | Meaning |
|---|---|---|
| `none` | none | No rotation (identity) |
| `quaternion_wxyz` | 4 | Unit quaternion, scalar first |
| `quaternion_xyzw` | 4 | Unit quaternion, scalar last |
| `euler_zyx_deg` | 3 | KUKA A, B, C: rotate about the tool z by A, then the new y by B, then the new x by C |
| `euler_xyz_deg` | 3 | Rotate about x by a, then the new y by b, then the new z by c |
| `fixed_xyz_deg` | 3 | FANUC W, P, R: rotate about the fixed base x, then y, then z |
| `rotvec_deg` | 3 | Rotation vector: direction is the axis, length is the angle in degrees |
| `matrix` | 9 | Row-major 3 by 3 rotation matrix |

Table 5. Rotation types of the pose log.

Example lines:

```
procedure,target_id,gap_mm,station_z_mm,field,pose_index,frames,x_mm,y_mm,z_mm,rotation_type,r1,r2,r3,r4
C,T5-S,15,750,0,17,10,812.40,-33.21,455.02,euler_zyx_deg,-91.3,19.8,0.4,
Z,T2,,750,0,42,10,812.43,-33.19,455.01,euler_zyx_deg,-91.3,19.8,0.4,
```

After the session, the engineer builds the manifest from the pose log, the capture folder, the plan, and the registration:

```
python3 -m sensorperf.cli.make_manifest --pose-log pose_log.csv --captures Characterization_20261014/ \
    --plan plan/poses.csv --registration registration.json --sensor-config-id <config_id>
```

The tool matches every file to its plan row by its name, takes the read-back pose from the log, and computes the target pose in the camera frame through the registration. It lists, by name, every capture file with no log row or plan row, every planned pose without captures, every logged frame without a file, and every pose read back far from the plan. A pose without files or a file without a pose is a warning; add `--strict` to make warnings errors. Problems in the log or plan stop the tool and no manifest is written. Fix what it names and run it again. Do not use `poses.csv` as the pose log for real captures: it holds the commanded poses, and the analysis needs the poses the robot reported. The manifest is the file the analysis reads; keep the pose log too.

What the manifest contains, one line per frame (Table 6):

| Group | Columns |
|---|---|
| Identity | `file`, `procedure`, `target_id`, `gap_mm`, `station_z_mm`, `field`, `pose_index`, `frame_index` |
| Randomization | `seed`, `offset_h_mm`, `offset_v_mm` |
| Robot pose read back (flange to base) | `robot_x_mm`, `robot_y_mm`, `robot_z_mm`, `robot_rx_deg`, `robot_ry_deg`, `robot_rz_deg` |
| Target pose in the camera frame, from registration (ground truth) | `target_x_mm`, `target_y_mm`, `target_z_mm`, `target_rx_deg`, `target_ry_deg`, `target_rz_deg` |
| Readings | `indicator_mm` (series Z only), `timestamp`, `sensor_temp_c`, `air_temp_c`, `ambient_ir`, `sensor_config_id` |
| Sub-series details | `subseries`, `tilt_axis`, `tilt_deg`, `step_mm`, `visit`, `level_index` |

Table 6. Manifest columns. Rotations are rotation vectors in degrees.

## 12. Checking the captures

Run the quick check as soon as a series is complete, while the robot and targets are still set up:

```
python3 -m sensorperf.cli.check_captures --session Characterization_20261014/ --out check.json
```

It prints one line per pose and a verdict: the number of frames, the valid fraction, border contact, and the fit of the front plane (and, where there is one, the back plane) against the registered target. Things it flags, and what they mean:

- Low valid fraction: fewer than half of the pixels the registered target should occupy were read. Check exposure, or the target was outside the field.
- Feature outline crossing the image border: the target is partly out of view, so a disk, cutout, or square is cut off. The pose is unusable; ask the engineer to re-plan it slightly inward.
- Registered target does not appear in the field of view: the manifest or the registration is wrong for that pose.
- Front or back plane fit residual too large: the plane was classified wrongly, or the pose is wrong.
- Front or back plane far from the registered plane: a wrong registration or a target that moved. The limits are loose on purpose: the sensor's own bias is what the analyses measure.
- Front or back plane normal disagrees with the registered normal by more than 2 degrees: a tilted mount or a wrong pose.

Re-capture flagged poses after fixing the cause. Do not delete the lines from the log; add corrected lines with a new `pose_index`. The tool's exit code is 0 when nothing is flagged, 1 when something is, and 2 when it cannot read the session. The limits are options of the tool (appendix B).

The tool also has a pilot option for series D (§10, step 1): `--pilot 750` prints the pilot detection counts instead of the check.

## 13. Data layout, deliverables, and the capture budget

Save every frame raw (`.mc`), with one manifest row per frame, so that any analysis can be re-run without re-capturing. Figure 8 shows how the folder feeds the analyses. The folder layout, under `Noise Estimation Data/`, is:

```
Characterization_<YYYYMMDD>/
  sensor_config.json   registration.json   targets_asbuilt.csv
  targets.json         parameters.json     manifest.csv
  session_log.md       environment_log.csv
  00_registration/  A_noise/  B_edges/  B_zstep/  C_area/  D_detect/  sentinels/
  analysis/            (written by the analyses)
```

![](figures/fig_data_flow.png)

Figure 8. The session folder feeds the six analyses, which write into `analysis/`. The fits of A and the boundary terms of E converge on `forward_model_parameters.json`.

**Deliverables checklist**

- [ ] `Characterization_<YYYYMMDD>/` with all `.mc` files in the sub-folders above, named by the rule of §5
- [ ] `sensor_config.json` with every filter setting and the dagger values filled in
- [ ] `registration.json`, accepted, with its residual noted in `session_log.md`
- [ ] `targets_asbuilt.csv` with the uncertainties
- [ ] `poses.csv`, `plan_summary.txt`, `plan.png`, `targets.json`, and `parameters.json` of the plan used, and the seed
- [ ] The pose log, and `manifest.csv` produced by `make_manifest` without errors
- [ ] `check.json` with no flags, or a note explaining each remaining flag
- [ ] `environment_log.csv` and the temperature logs
- [ ] `session_log.md` with: date, sensor serial number, warm-up time, settle time, base frame name, plate flatness and finish, mount-check results, target changes with times, and anything unusual
- [ ] Photos of the setup: sensor stand, each target on the adapter, the dial indicator

**Capture budget.** Table 7 is computed from the default plan. The estimate assumes 10 frames per second and 3 s per move plus settle. Both are assumptions; the VSX3000 frame rate in the chosen trigger mode should replace them. Target swaps, warm-up, and the D pilot analysis are excluded. Allow two to three working days in total.

| Series | Poses | Frames | Robot time (h) |
|---|---:|---:|---:|
| Registration (R) | 30 | 300 | 0.03 |
| A noise | 80 | 6,800 | 0.26 |
| B-HV edges | 520 | 15,600 | 0.87 |
| B-Z depth steps | 573 | 5,730 | 0.64 |
| C area | 1,360 | 13,600 | 1.51 |
| D detection | 8,160 | 8,160 | 7.03 |
| Sentinels | 12 | 360 | 0.02 |
| **Total** | **10,735** | **50,550** | **10.3** |

Table 7. Capture budget of the default plan: poses, frames, and robot time per series.

The plan has 10,735 poses and 50,550 frames, about 10.3 hours of robot time. The extended 0 percent series is the largest part of it. Check storage before starting: multiply the size of one `.mc` frame by 50,550 frames.

## 14. Running the analyses

The engineer runs the analyses on the finished session folder. One command runs them all, in the order A, B, Z, C, D, E, because A supplies the noise level that B, C, and D use, and B supplies the edge spread function that C compares with:

```
python3 -m sensorperf.cli.analyze --session Characterization_20261014/ [--only A B Z C D E]
```

`--only` runs a subset (the letters are those of Table 4, with B for the lateral analysis and Z for the depth analysis). An analysis whose frames are absent from the manifest is skipped with a notice. The results go into `analysis/` in the session folder; an optional `--out DIR` changes the folder. The exit code is 0 when every selected analysis ran, 1 when one failed (the others still run), and 2 when the session cannot be read. Each analysis writes a CSV summary, a detail file, and figures (PNG and SVG). The definitions are in the specification, Sections 10 to 14; this section says what each one reports.

**A, noise versus Z** (specification Section 10; series A and the sentinels). Reports the temporal, fixed-pattern, and total depth noise of the plate at each Z, with the bias, the fill rate, the spatial correlation length, and the depth quantization step, and fits the disparity-noise model. It also reports how noise and fill rate change with incidence angle (the tilt sub-series) and corrects for drift with the sentinels. Writes `A_noise_summary.csv` (one row per station) and figures: the noise curves with the model fit, fill rate, noise maps, autocorrelation profiles, and depth-code histograms. The fitted disparity noise, the noise floor, the quantum, k, and the correlation length go into `forward_model_parameters.json`.

**B-HV, resolution in H and V** (specification Section 11.1; series B). Reports the 10 to 90 percent rise distance and the MTF50 of the depth edge response, for each Z, edge orientation (H or V), polarity, and gap, from two cross-checked estimates (the slanted edge and the robot-stepped poses). It reports whether the result depends on step height, and the edge offset that analysis E uses. Writes tables and figures in `analysis/`; confidence intervals come from bootstrapping over poses.

**B-Z, resolution in Z** (specification Section 11.2; series Z). Reports the smallest commanded Z step detected with 50 percent probability for square patches of side 1, 5, 20 px, the step-response gain and its linearity, and the measured quantum from the fine staircase. The step truth is the dial-indicator reading. Writes tables and figures in `analysis/`.

**C, true versus sensed area** (specification Section 12; series C). Reports the sensed area of disks and cutouts at the half-height contour compared with the true (as-built) area and with the geometrically visible area, against the diameter in pixels, and the edge bias in mm and px. It also compares with the prediction from the edge response of B, and the field and open-background variants. Writes `C_area_summary.csv` and figures: transfer curves, edge bias against Z, and phase spread against diameter in pixels.

**D, minimum detectable size** (specification Section 13; series D). Reports the diameter and subtended angle at 50 percent, 10 percent, and 0 percent detection probability, corrected for false alarms, for disks and cutouts, for each Z and configuration, with confidence intervals. The 0 percent value is a bound that is shown, with 0.95 confidence, to be below 0.01, not a proof of zero. It also checks the independence of the trials. Writes `D_detect_summary.csv` (one row per configuration and Z) and figures: psychometric curves and angle against Z.

**E, boundary detection bias** (specification Section 14; reuses the B and C frames, no captures of its own). Reports, near a true edge, how often the sensor reports a value where geometry says no stereo read is possible, how often it reports a no-read where a surface was visible, and whether it favors the near or the far surface. Writes `E_boundary_bias.csv` and figures: outcome profiles against distance to the edge, and the bias indices against Z. The widths and the near-far preference go into `forward_model_parameters.json`.

`forward_model_parameters.json` is assembled after the selected analyses, from the terms each one measured. It is the hand-off to the Tier-A forward-noise simulator.

## 15. Things that spoil a session

The smallest Z steps and the absolute bias are the measurements most limited by the setup. The relative measurements are robust: noise, rise distance, transfer-curve shape, and detection minimums. Table 8 lists the sources of error. The magnitudes are typical values from the specification, not measured ones; the engineer replaces them with measured values after §4.

| Source | Typical magnitude | Affects | What you do about it |
|---|---|---|---|
| Robot repeatability | 0.02 to 0.05 mm | Smallest Z steps; edge position | Read the dial indicator; the plan averages over phase poses |
| Robot absolute accuracy | 0.2 to 1 mm over large moves | Bias, registration | Registration over many poses; tests run as local moves from registered stations |
| Registration residual | Acceptance limit 0.15 mm RMS | Bias, edge offset, small-feature area | Do not skip the acceptance gate; report the residual with every result |
| IR fiducial depth accuracy at 1 m | A few tenths of a mm (rough estimate) | Absolute bias only | Many poses; plate pattern known to 0.02 mm |
| As-built diameter uncertainty | Set by the measuring instrument | True area (for example 8 percent at 0.5 mm with 0.02 mm uncertainty) | Measure with the comparator or microscope and record the uncertainty |
| Plate flatness | 0.05 mm | Fixed-pattern noise, bias | Keep the flatness report; map the plate on a CMM if available |
| Thermal drift | Unknown until the sentinels run | Bias, series Z | Warm-up gate, sentinels, A, B, A, B order, randomized order |
| Front-back interreflection | Unknown | Cutout and small-gap results | Matte finish; both gaps are captured |
| Sensor temporal filter | Depends on the configuration | Noise (underestimated), trial independence | Discard frames after each move until the plane settles (§3, step 4); filters-off repeat if possible |
| Reflectance mismatch | Avoided by design | Edges, area | One finish on all surfaces |

Table 8. Sources of error, their size, and what to do about them.

Things that spoil a session in practice:

- Moving or bumping the sensor. If it happens, everything after it is a new session.
- Changing exposure, gain, any filter, the base frame, or the lighting mid-session.
- Using the commanded pose instead of the reported pose in the log.
- Opening the enclosure during a series, or a change in ambient IR.
- Skipping the warm-up or the settle check, or shortening the settle wait.
- A loose adapter, a spacer that has moved, or a target that has shifted on its dowels: do the mount check (§4, step 7) after every swap.
- Fingerprints, dust, or gloss on a target: wipe with isopropyl alcohol. A shiny spot returns a bright highlight and a bad read.
- Different finish on the front and back plates.
- Using two frames of one pose as two detection trials in series D (§10, step 5).
- Not recording a filter setting or a configuration change in `sensor_config.json`.
- Losing the seed of the plan. Without it the offsets cannot be reproduced; the plan files keep it.

Limitations. Results hold for one surface finish, static targets, mostly fronto-parallel poses, and the sensor configuration recorded in `sensor_config.json`. If the IR fiducial registration is not possible (§4, step 3), a constant Z offset is absorbed into the registration and absolute bias is not measurable. Z-scale and nonlinear bias still are, because a rigid transform cannot absorb them. A "0 percent" minimum means below the stated bound with 0.95 confidence, not proven zero.

---

## Appendix A. Parameters

Every arbitrary constant in the procedure is a named parameter. The table below is generated from the code (`sensorperf/parameters.py`), so it always shows the values the planner and the analyses use. The values are starting points for the VSX3000 at 500 to 1000 mm.

Values marked with a dagger in the specification depend on VSX3000 datasheet or SDK values that were not available when it was written. They are not in this table: the code leaves them empty until §3, step 5 fills them in `sensor_config.json`, and any computation that needs one stops with a clear message instead of guessing. They are: the left IR focal lengths (`SENSOR_FX_PX`, `SENSOR_FY_PX`) and principal point (`SENSOR_CX_PX`, `SENSOR_CY_PX`), the stereo baseline (`SENSOR_BASELINE_MM`), the projector offset (`PROJECTOR_OFFSET_MM`), the depth LSB (`DEPTH_LSB_MM`), and the frame rate in the chosen trigger mode. Where this document quotes a number that depends on them (the diameter range, the field fit, the budget), it uses indicative values for a 640 by 480 sensor and says so. These are not datasheet values.

| Name | Value | Meaning |
|---|---|---|
| `Z_MIN_MM` | 500 | Near range limit. |
| `Z_MAX_MM` | 1000 | Far range limit. |
| `Z_NOISE_STEP_MM` | 50 | Spacing of the noise stations (A); 500 to 1000 in steps of 50 gives 11 stations. |
| `Z_SHAPE_STATIONS_MM` | 500, 625, 750, 875, 1000 | Stations for the edge, area and detection tests (B, C, D). |
| `Z_REDUCED_STATIONS_MM` | 500, 750, 1000 | Subset of stations for the slowest tests (B-Z, the D 0 percent series, the A tilt sub-series). |
| `Z_REFERENCE_MM` | 750 | The mid-range station used for sentinels, the warm-up check, the re-mount check, the field sub-series and the D pilot (the document writes 750 mm in each place). |
| `FIELD_OFFSET_FRACTION` | 0.6 | Off-axis field positions at (+/- f W/2, +/- f H/2) from the center, f of the half field (A, C). |
| `TILT_ANGLES_DEG` | 0, 15, 30, 45 | Plate tilts about H and about V in the A tilt sub-series. |
| `WARMUP_MIN_MINUTES` | 45 | Minimum powered time before any capture. |
| `WARMUP_DRIFT_WINDOW_MIN` | 10 | Window over which warm-up drift is judged. |
| `WARMUP_DRIFT_FRACTION_OF_SIGMA` | 0.1 | Allowed drift of the mean plane Z over the window, as a fraction of sigma_t at that Z. |
| `WARMUP_CHECK_INTERVAL_MIN` | 1 | Interval of the warm-up captures (Step 4.3: 10 frames every minute). |
| `WARMUP_CHECK_FRAMES` | 10 | Frames per warm-up capture (Step 4.3). |
| `DRIFT_SENTINEL_INTERVAL_MIN` | 60 | Interval between drift sentinel captures. |
| `SENTINEL_FRAMES` | 30 | Frames per drift sentinel (Section 5, Step 3). |
| `ROBOT_SETTLE_TIME_S` | 2 | Wait after motion before a capture; Step 4.4 verifies it. |
| `SETTLE_CHECK_FRAMES` | 100 | Frames of each settle-and-vibration capture (Step 4.4). |
| `SETTLE_SIGMA_EXCESS_FRACTION` | 0.1 | Servo-on sigma_t may exceed brakes-on sigma_t by at most this fraction (Step 4.4). |
| `FRAMES_PER_NOISE_STATION` | 100 | Frames per Z, field and tilt pose in A (the tilt sub-series uses frames_per_tilt_pose). |
| `FRAMES_PER_TILT_POSE` | 50 | Frames per pose of the A tilt sub-series (Section 5, Step 5). |
| `FRAMES_PER_EDGE_POSE` | 30 | Frames per edge pose (B-HV). |
| `FRAMES_PER_ZSTEP_POSE` | 10 | Frames per Z-step pose (B-Z). |
| `FRAMES_PER_AREA_POSE` | 10 | Frames per area pose (C). |
| `FRAMES_PER_DETECTION_TRIAL` | 1 | Frames per detection trial (D): one, giving single-frame detectability. |
| `FRAMES_PER_REGISTRATION_POSE` | 10 | Frames per registration pose (Section 4; the document's REGISTRATION_FRAMES). |
| `MOVE_AND_SETTLE_TIME_S` | 3 | Robot move plus settle time per pose assumed by the capture budget (Section 9). |
| `PHASE_JITTER_SPAN_PX` | 8 | Square span of the random lateral offsets in pixels at the station depth (B, C, D). |
| `PHASE_JITTER_POSES_EDGE` | 25 | Random-offset poses per edge station (B-HV). |
| `PHASE_JITTER_POSES_AREA` | 30 | Random-offset poses per area configuration (C). |
| `FIELD_SUBSERIES_POSES_AREA` | 10 | Random-offset poses per array at each off-axis field position (Section 7, Step 3). |
| `Z_STEP_LADDER_MM` | 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 4 | Commanded step sizes of the B-Z ladder. |
| `Z_STEP_REPEATS` | 10 | ABAB cycles for each step size. |
| `Z_STAIRCASE_SUBDIVISION` | 10 | Fine-sweep points per expected depth quantum. |
| `Z_STAIRCASE_QUANTA` | 3 | The fine staircase sweeps from Z0 to Z0 plus this many expected quanta (Section 6.2, Step 3). |
| `Z_STAIRCASE_FRAMES` | 10 | Frames per staircase step. |
| `ZSTEP_PATCH_SIZES_PX` | 1, 5, 20 | Side lengths of the square patches of the B-Z analysis (1 px, 5 x 5, 20 x 20). |
| `DETECTION_FALSE_ALARM_TARGET` | 0.01 | Target false-alarm rate per window; sets the threshold tau from the blank-site distribution. |
| `DETECTION_MIN_CONNECTED_PX` | 2 | Minimum connected region counted as a detection. |
| `DETECTION_WINDOW_MARGIN_PX` | 3 | Search window radius equals D/2 in pixels plus this margin. |
| `DETECTION_LEVELS` | 9 | Diameter levels per psychometric curve. |
| `DETECTION_LEVEL_LOW_FACTOR` | 0.3 | Smallest level as a multiple of the pilot D_50. |
| `DETECTION_LEVEL_HIGH_FACTOR` | 2.5 | Largest level as a multiple of the pilot D_50. |
| `DETECTION_TRIALS_PER_LEVEL` | 60 | Independent trials per level in the main D series. |
| `DETECTION_ZERO_TRIALS` | 300 | Trials per level in the extended 0 percent series. |
| `DETECTION_ZERO_PROBABILITY_BOUND` | 0.01 | Upper bound on detection probability that defines practically zero detection. |
| `DETECTION_LAPSE_RATE_MAX` | 0.05 | Upper bound of the lapse rate lambda in the psychometric fit (Section 13, Step 4). |
| `DETECTION_FINE_LADDER_RATIO` | 1.189 | Diameter ratio of a dedicated fine detection plate (Section 8, Step 2). |
| `CONFIDENCE_LEVEL` | 0.95 | Level of all confidence intervals and bounds (B, C, D, E). |
| `BOOTSTRAP_RESAMPLES` | 2000 | Resamples for bootstrap confidence intervals (C, D, E). |
| `INDEPENDENCE_SIGMA_MULTIPLE` | 2 | Lag-1 autocorrelation of a detection sequence must lie within this many standard errors (1/sqrt(n)) of zero (Section 13, Step 1: +/- 2/sqrt(n)). |
| `BOUNDARY_BAND_HALF_WIDTH_PX` | 8 | Analysis band on each side of a true edge (E); also the ROI shrink in A and the reference-plane exclusion in B and C. |
| `BOUNDARY_BIN_WIDTH_PX` | 0.25 | Signed-distance bin width (B, E): four bins per pixel. |
| `SURFACE_ASSIGNMENT_SIGMA_MULTIPLE` | 3 | A read within this many sigma_tot(Z) of a reference plane is assigned to that plane (E). |
| `FRONT_READ_HEIGHT_THRESHOLD` | 0.5 | Normalized height h above which a read counts as a front read (C, Section 12, Step 3). |
| `ESF_RISE_LOW` | 0.1 | Lower normalized height of the rise distance (10 percent). |
| `ESF_RISE_HIGH` | 0.9 | Upper normalized height of the rise distance (90 percent). |
| `ESF_HALF_HEIGHT` | 0.5 | Normalized height of the edge crossing s_50 (B-HV, Step 8) and of the area iso-contour (C, Step 5). |
| `ESF_LINEARITY_TOLERANCE_H` | 0.1 | Largest allowed difference in h between the small-gap and large-gap ESFs before the result is marked step-height dependent (Section 11.1, Step 7). |
| `ESF_AGREEMENT_BINS` | 1 | The robot-stepped and slanted-edge ESFs must agree within this many bins once aligned (Step 5). |
| `NOISE_CLOSURE_TOLERANCE` | 0.2 | Allowed relative mismatch of sigma_tot^2 against sigma_t^2 + sigma_fp^2 + bias^2 (Section 10, Step 5). |
| `QUANTIZATION_PATCH_PX` | 20 | Side of the central patch whose depth codes are histogrammed (Section 10, Step 8). |
| `LEGACY_BOX_HALF_PX` | 10 | Half side of the legacy 20 x 20 px boxes of testZRepeatabilityBrownBoard.py (Step 12). |
| `LEGACY_BOX_CENTERS_PX` | (264, 253), (137, 81), (401, 83), (404, 386), (129, 392) | The five VSX3000 BrownBoard box centers (column, row) of testZRepeatabilityBrownBoard.py. |
| `LEGACY_METRIC_DEPTHS_MM` | 700, 1000 | Depths at which the legacy metrics are computed (Section 10, Step 12). |
| `AUTOCORRELATION_THRESHOLD` | 0.3679 | The correlation length is the lag where the normalized autocorrelation first falls to this (1/e). |
| `NOISE_PLATE_SIZE_MM` | 400, 400 | Uniform matte plate (T2), width x height. |
| `PLATE_FLATNESS_MM` | 0.05 | Required flatness of every plate. |
| `PLATE_FLATNESS_SIGMA_FRACTION` | 0.25 | The flatness must be at most this fraction of the smallest expected sigma_tot. |
| `GAP_SMALL_MM` | 15 | Small front-to-back plate distance G. |
| `GAP_LARGE_MM` | 60 | Large front-to-back plate distance G. |
| `EDGE_SLANT_DEG` | 5 | Rotation of the edge target in its own plane relative to the image axes (B-HV). |
| `EDGE_SQUARE_SIZE_MM` | 160 | Side of the raised square (T3a) and of the square window (T3b). |
| `CHAMFER_MARGIN_DEG` | 10 | Margin added to the worst-case ray angle in the bevel check (Section 3.3). |
| `EDGE_LAND_MAX_MM` | 0.2 | Maximum residual flat land at a knife edge. |
| `DIAMETER_LADDER_RATIO` | 1.414 | Ratio between successive diameters of the disk and cutout ladder. |
| `DIAMETER_MIN_FOOTPRINT_FRACTION` | 0.3 | Smallest diameter as a fraction of the pixel footprint at Z_MIN. |
| `DIAMETER_MAX_FOOTPRINT_MULTIPLE` | 30 | Largest diameter as a multiple of the pixel footprint at Z_MAX. |
| `FEATURE_ISOLATION_PX` | 30 | Minimum edge-to-edge spacing between features, pixels at Z_MIN. |
| `POST_DIAMETER_FRACTION_OF_D0` | 0.5 | Disk support posts must be thinner than this fraction of the pilot D_0. |
| `FRAME_CHECK_PX` | 0.5 | IR-edge to depth-discontinuity agreement required in Step 4.5. |
| `REGISTRATION_POSES` | 30 | Hand-eye poses spanning the volume (Section 4, Step 6). |
| `REGISTRATION_TILT_RANGE_DEG` | 20 | Half range of the registration tilts about H and V (+/-). |
| `REGISTRATION_RESIDUAL_ACCEPT_MM` | 0.15 | Acceptance limit of the registration residual, mm RMS. |
| `MOUNT_CHECK_DEPTH_MM` | 750 | Depth of the once-per-mount plane-fit check (Section 4, Step 8). |
| `CONTINUOUS_ANGLE_STEP_MM` | 10 | Z step of the optional continuous-angle D variant (Section 8, Step 7). |
| `CONTINUOUS_ANGLE_POSES_PER_STEP` | 20 | Random-offset poses per Z step of that variant. |

Table 9. The parameters of the procedure, with the values of the default plan.

## Appendix B. Software reference

The code that supports this procedure is the Python package `sensorperf/` in this repository. The command-line tools each import other modules of the package, so they cannot be copied out on their own. Ship the whole `sensorperf/` directory together with `pyproject.toml`, `requirements.txt`, and `tests/`, either as a clone of the repository or as a copy of those items with the directory layout kept.

The tools: `plan_stations` (§5), `register` (§4), `make_manifest` (§11), `check_captures` (§10 and §12), `simulate` (a synthetic session for practice, appendix C), and `analyze` (§14). Table 10 lists the package files with their line counts and what they do. The design document `docs/design/code_design.md` describes the modules.

| File | Lines | What it does |
|---|---:|---|
| `sensorperf/__init__.py` | 16 | sensorperf: code for the VSX3000 Resolution, Area-Fidelity, Detectability, and Noise Characterization Procedure. |
| `sensorperf/acquisition/__init__.py` | 1 | sensorperf.acquisition: see the package docstring and docs/design/code_design.md. |
| `sensorperf/acquisition/check.py` | 598 | Quick-look check of a capture set (Part I of the procedure) and the D pilot detection counts (Section 8, Step 1; Section 13, Step 2). |
| `sensorperf/acquisition/plan.py` | 1308 | Station and pose planning for Part I of the procedure (Sections 4 to 9). |
| `sensorperf/acquisition/pose_log.py` | 531 | Robot pose log -> capture manifest (document, Section 9). |
| `sensorperf/analysis/__init__.py` | 1 | sensorperf.analysis: see the package docstring and docs/design/code_design.md. |
| `sensorperf/analysis/area.py` | 1082 | Analysis C: true versus sensed area (procedure document, Section 12), Steps 1 to 12. |
| `sensorperf/analysis/boundary.py` | 869 | Analysis E: boundary detection bias (procedure document, Section 14), Steps 1 to 8. |
| `sensorperf/analysis/common.py` | 407 | What every analysis of Part II shares: the registered geometry of a pose (which pixel should see which surface, where the true edges are), the reference planes, |
| `sensorperf/analysis/detection.py` | 971 | Analysis D: minimum detectable size at 50, 10 and 0 percent (procedure document, Section 13), Steps 1 to 11. |
| `sensorperf/analysis/forward_model.py` | 73 | Assembly of ``forward_model_parameters.json`` (document, Section 1 and Section 10 Step 13, Section 14 Step 8): the hand-off from the analyses to the Tier-A forw |
| `sensorperf/analysis/noise.py` | 1101 | Analysis A: noise versus Z (procedure document, Section 10), on the frames of procedure "A" (the noise-plate series) and the drift sentinels (procedure "S", Ste |
| `sensorperf/analysis/resolution_depth.py` | 702 | Analysis B-Z: effective resolution in depth (procedure document, Section 11.2), from the Z-step series (procedure "Z"): the noise plate T2 at a station Z0, a st |
| `sensorperf/analysis/resolution_lateral.py` | 781 | Analysis B-HV: effective lateral resolution in H and V (procedure document, Section 11.1), from the edge series (procedure "B"): the raised square T3a and the s |
| `sensorperf/cli/__init__.py` | 1 | sensorperf.cli: see the package docstring and docs/design/code_design.md. |
| `sensorperf/cli/analyze.py` | 98 | Command line: run the Part II analyses on a session folder. |
| `sensorperf/cli/check_captures.py` | 122 | Command line: a quick-look check of a capture session before the long analyses. |
| `sensorperf/cli/make_manifest.py` | 117 | Command line: build the capture manifest (procedure document, Section 9) from the robot's pose log, the capture files and the plan. |
| `sensorperf/cli/plan_stations.py` | 185 | Command line: plan the stations and poses of the characterization capture (procedure document, Sections 4 to 9). |
| `sensorperf/cli/register.py` | 184 | Command line: solve the robot-to-sensor registration (procedure document, Section 4, Steps 6 and 7) from the registration observations. |
| `sensorperf/cli/simulate.py` | 102 | Command line: write a synthetic characterization session. |
| `sensorperf/features/__init__.py` | 1 | sensorperf.features: see the package docstring and docs/design/code_design.md. |
| `sensorperf/features/depth_features.py` | 210 | Per-pixel features computed from depth images: temporal statistics over the frames of one pose, the local surface slopes (s_u, s_v) that are inputs of the corre |
| `sensorperf/features/normals.py` | 97 | Surface normals from a least-squares plane fit over an N x N window of camera-frame points (the downstream 5 x 5 estimator, D-8). |
| `sensorperf/features/planes.py` | 145 | Plane fits and plane-based depth references shared by the analyses. |
| `sensorperf/geometry/__init__.py` | 1 | sensorperf.geometry: see the package docstring and docs/design/code_design.md. |
| `sensorperf/geometry/camera.py` | 89 | Pinhole camera model of the depth sensor, built from a capture file's header. |
| `sensorperf/geometry/registration.py` | 324 | Robot-to-sensor registration (document, Section 4): the two transforms that turn a read-back robot pose into a ground-truth target pose in the camera frame, the |
| `sensorperf/geometry/targets.py` | 716 | The two-plane targets of the characterization procedure (document, Section 3): a front surface with knife-edge features standing a gap G in front of a back plat |
| `sensorperf/geometry/transforms.py` | 95 | Rigid transforms and the rigid fit between two point sets. |
| `sensorperf/io/__init__.py` | 1 | sensorperf.io: see the package docstring and docs/design/code_design.md. |
| `sensorperf/io/capture_set.py` | 113 | Frames of one commanded pose loaded together as a stack (adapted from the calibration repository's capture_set.py for the characterization manifest). |
| `sensorperf/io/manifest.py` | 451 | The capture manifest of the characterization procedure (document, Section 9): one row per captured frame, saying which procedure and target it belongs to, where |
| `sensorperf/io/matcloud.py` | 404 | Reader/writer for Liberty Reach's ".mc" ("Matrix Cloud") file format. |
| `sensorperf/io/qt_datastream.py` | 433 | A minimal reader/writer for Qt5's ``QDataStream`` binary encoding (default stream version, which is what ``MC::toFile``/``MC::fromFile`` use -- see ``MC.cpp`` i |
| `sensorperf/io/session.py` | 140 | The session folder of Section 9 and the small JSON records it holds. |
| `sensorperf/parameters.py` | 487 | Every arbitrary constant of the characterization procedure, as a named parameter (procedure document, Section 2), plus the five geometric relations of that sect |
| `sensorperf/simulate/__init__.py` | 1 | sensorperf.simulate: see the package docstring and docs/design/code_design.md. |
| `sensorperf/simulate/demo_plan.py` | 300 | A small but complete demonstration plan for the synthetic session writer (Sections 4 to 9 of the procedure, drastically reduced), a plausible registration to re |
| `sensorperf/simulate/sensor_model.py` | 399 | INDICATIVE synthetic depth renderer of a :class:`~sensorperf.geometry.targets.TwoPlaneTarget` (design document, Section 5, "simulate/sensor_model.py"). |
| `sensorperf/simulate/session.py` | 177 | Write a synthetic characterization session (design document, Section 5, "simulate/session.py"): render every :class:`~sensorperf.acquisition.plan.PlannedCapture |
| `sensorperf/stats/__init__.py` | 1 | sensorperf.stats: see the package docstring and docs/design/code_design.md. |
| `sensorperf/stats/intervals.py` | 143 | Binomial and bootstrap confidence intervals. |
| `sensorperf/stats/psychometric.py` | 549 | Psychometric curves, model-free isotonic thresholds and the threshold (floor) model of the detectability analysis. |

Table 10. The files of the package.

Output of `python3 -m sensorperf.cli.plan_stations --help`:

```
usage: build.py [-h] --out DIR [--registration PATH] [--sensor-config PATH]
                [--parameters PATH] [--series LETTER [LETTER ...]]
                [--pilot-d50-mm VALUE [VALUE ...]]
                [--pilot-d0-mm VALUE [VALUE ...]] [--seed N] [--no-extended]
                [--filters-off] [--open-background]

Plan the stations and poses of the characterization capture (Sections 4 to 9):
registration, A noise, B-HV edges, B-Z depth steps, C area, D detection, with
logged randomization and drift sentinels. Writes poses.csv, plan_summary.txt,
plan.png, targets.json and parameters.json.

options:
  -h, --help            show this help message and exit
  --out DIR             output directory (created if needed)
  --registration PATH   registration.json; adds the flange pose to command
                        (robot base frame) to poses.csv
  --sensor-config PATH  sensor_config.json with the sensor geometry; without
                        it the indicative geometry is used
  --parameters PATH     parameters.json with overrides of the Section 2
                        parameter table
  --series LETTER [LETTER ...]
                        subset of series to plan, by procedure letter: R A B Z
                        C D (R registration, A noise, B edges, Z depth steps,
                        C area, D detection); default all
  --pilot-d50-mm VALUE [VALUE ...]
                        pilot D_50 in mm (Section 8, Step 1): one number for
                        every array, or TARGET@GAP=VALUE entries; used only
                        for the summary and a warning
  --pilot-d0-mm VALUE [VALUE ...]
                        pilot D_0 in mm, same syntax as --pilot-d50-mm
  --seed N              master random seed; every shuffle and offset draws
                        from it and is logged (default 0)
  --no-extended         skip the extended 0 percent trials of the D series
                        (Section 8, Step 4)
  --filters-off         append the filters-off repeat of the A series (Section
                        5, Step 7)
  --open-background     add the open-background variant of the C series for
                        the cutout arrays (Section 7, Step 4)
```

Output of `python3 -m sensorperf.cli.register --help`:

```
usage: build.py [-h] --observations CSV [--method {fiducial,planes}]
                [--accept-mm MM] [--out PATH]

Solve the robot-to-sensor registration (camera_to_base, target_to_flange) from
registration observations: the read-back flange poses with the fiducial
solve's target poses (hand-eye) or with depth-plane fits. Writes
registration.json and prints the residual and the accept verdict.

options:
  -h, --help            show this help message and exit
  --observations CSV    one row per registration pose: pose_id, x_mm, y_mm,
                        z_mm, rotation_type, r1..r9, and either tx_mm, ty_mm,
                        tz_mm, trx_deg, try_deg, trz_deg (rotation vector in
                        degrees) or nx, ny, nz, distance_mm
  --method {fiducial,planes}
                        fiducial: hand-eye from full target poses; planes:
                        from depth-plane fits only (default fiducial)
  --accept-mm MM        acceptance limit of the RMS residual in mm (default:
                        REGISTRATION_RESIDUAL_ACCEPT_MM, 0.15)
  --out PATH            registration file to write (default registration.json)
```

Output of `python3 -m sensorperf.cli.make_manifest --help`:

```
usage: build.py [-h] --pose-log PATH --captures DIR --plan PATH --registration
                PATH [--sensor-config-id TEXT] [--out PATH] [--strict]

Build the capture manifest (manifest.csv) from a robot pose log, the capture
files and the plan (poses.csv of plan_stations), through the registration.

options:
  -h, --help            show this help message and exit
  --pose-log PATH       robot pose log CSV: one row per frame (file, x_mm,
                        y_mm, z_mm, rotation_type, r1..r9) or one row per pose
                        (procedure, target_id, gap_mm, station_z_mm, field,
                        pose_index, frames, x_mm, ...); rotation_type: none,
                        quaternion_wxyz, quaternion_xyzw, euler_zyx_deg,
                        euler_xyz_deg, fixed_xyz_deg, rotvec_deg, matrix
  --captures DIR        session root or folder holding the .mc files (sub-
                        folders are searched)
  --plan PATH           poses.csv written by plan_stations
  --registration PATH   registration.json (camera_to_base and
                        target_to_flange)
  --sensor-config-id TEXT
                        identifier of the sensor configuration, written to
                        every row
  --out PATH            manifest file to write (default: manifest.csv in the
                        --captures folder)
  --strict              treat warnings (unmatched files or plan rows, frame
                        count mismatches, poses read back far from the plan)
                        as errors
```

Output of `python3 -m sensorperf.cli.check_captures --help`:

```
usage: build.py [-h] --session DIR [--out PATH]
                [--min-valid-fraction MIN_VALID_FRACTION]
                [--border-margin-px BORDER_MARGIN_PX]
                [--plane-residual-warn-mm PLANE_RESIDUAL_WARN_MM]
                [--pose-residual-warn-mm POSE_RESIDUAL_WARN_MM]
                [--normal-warn-deg NORMAL_WARN_DEG]
                [--classification-margin-px CLASSIFICATION_MARGIN_PX]
                [--min-plane-pixels MIN_PLANE_PIXELS] [--pilot Z_MM]
                [--pilot-subseries LABEL [LABEL ...]]

Quick-look check of a capture session: valid fraction, border contact and the
front and back plane fits against the registered target, or (with --pilot) the
D pilot detection counts.

options:
  -h, --help            show this help message and exit
  --session DIR         session folder with sensor_config.json, targets.json
                        and manifest.csv
  --out PATH            write a JSON report here
  --min-valid-fraction MIN_VALID_FRACTION
                        flag a pose in which fewer than this fraction of the
                        expected target pixels were read
  --border-margin-px BORDER_MARGIN_PX
                        reads and feature outlines within this many pixels of
                        the border count as border contact
  --plane-residual-warn-mm PLANE_RESIDUAL_WARN_MM
                        flag a plane whose fit RMS exceeds this
  --pose-residual-warn-mm POSE_RESIDUAL_WARN_MM
                        flag a plane whose distance to the registered plane
                        exceeds this
  --normal-warn-deg NORMAL_WARN_DEG
                        flag a plane whose normal differs from the registered
                        one by more than this
  --classification-margin-px CLASSIFICATION_MARGIN_PX
                        erosion of the expected front and back masks before a
                        plane is fitted
  --min-plane-pixels MIN_PLANE_PIXELS
                        fewest pixels a plane fit is attempted with
  --pilot Z_MM          print the D pilot detection counts for the C poses at
                        this station (mm) instead of the check
  --pilot-subseries LABEL [LABEL ...]
                        sub-series of the C poses used by --pilot (default:
                        jitter)
```

Output of `python3 -m sensorperf.cli.simulate --help`:

```
usage: python3 -m sensorperf.cli.simulate [-h] --out OUT
                                          [--series {R,A,B,Z,C,D} [{R,A,B,Z,C,D} ...]]
                                          [--quick] [--seed SEED]
                                          [--frame-scale FRAME_SCALE]

Write a synthetic (INDICATIVE) characterization session with a demonstration
plan.

options:
  -h, --help            show this help message and exit
  --out OUT             session folder to write
  --series {R,A,B,Z,C,D} [{R,A,B,Z,C,D} ...]
                        series to simulate (default: all of R, A, B, Z, C, D)
  --quick               160 x 120 sensor with the same field of view and 0.2
                        of the frames
  --seed SEED           seed of the random generator
  --frame-scale FRAME_SCALE
                        fraction of the planned frames per pose to render
                        (default 1, or 0.2 with --quick)
```

The analysis tool is run as `python3 -m sensorperf.cli.analyze --session DIR [--only A B Z C D E]` (§14).

## Appendix C. Installing and running the software, step by step

These steps follow the same pattern on Linux and Windows. Allow about 20 minutes, most of it download time. Nothing here needs administrator rights.

C.1 Install Python. Python 3.10 or newer is required.

- Windows: download the installer for the latest Python 3 from python.org/downloads and run it. On the first screen tick "Add python.exe to PATH" before clicking Install. When it finishes, open a new Command Prompt (Start menu, type `cmd`) and type `python --version`; it must print `Python 3.10` or higher. If it prints nothing or an error, the PATH box was not ticked: run the installer again and choose Modify.
- Linux (Debian or Ubuntu): in a terminal, `sudo apt install python3 python3-venv python3-pip`, then `python3 --version`.

On Windows the Python command is `python`; on Linux it is `python3`. The commands below are written with `python3`; on Windows type `python` instead. Everything else is identical.

C.2 Get the code. Either unzip the archive the engineer sent, or, if `git` is installed, clone the repository. Put it somewhere without spaces in the path, for example `C:\perf\sensor_performance_estimation` on Windows or `~/perf/sensor_performance_estimation` on Linux. Inside that folder you must see `pyproject.toml`, `requirements.txt`, the folder `sensorperf`, and the folder `tests`. That folder is called the repository folder below.

C.3 Open a terminal in the repository folder. Windows: in File Explorer, open the repository folder, click in the address bar, type `cmd`, and press Enter; a Command Prompt opens already in that folder. Linux: `cd ~/perf/sensor_performance_estimation`. Check with `dir` (Windows) or `ls` (Linux) that `pyproject.toml` is listed; if it is not, you are in the wrong folder and every later step will fail with "No module named sensorperf".

C.4 Make a private Python environment and install into it. This keeps the tools' packages separate from anything else on the computer. In the terminal from C.3:

```
python3 -m venv .venv
```

then activate it, which you must do again in every new terminal before using the tools:

```
.venv\Scripts\activate          (Windows)
source .venv/bin/activate       (Linux)
```

The prompt now starts with `(.venv)`. Then install the package and everything it needs (numpy, scipy, matplotlib, pytest; about 150 MB, downloaded from the internet):

```
python3 -m pip install -e ".[figures,test]"
```

The `-e` installs the code in place, so the tools can be run from any folder once the environment is active, and a corrected file from the engineer takes effect by simply replacing it. If the computer has no internet, the engineer can supply the packages as files; ask. The engineer who rebuilds this document also needs the `docs` extra (`".[figures,test,docs]"`).

C.5 Run the self-test. Still in the repository folder:

```
python3 -m pytest -q
```

After some time it must end with a line like `N passed in M s`. Any line containing `FAILED` or `ERROR` means the installation is not right; copy the whole output into a text file and send it to the engineer. Do not start capturing until this passes.

C.6 Practice on synthetic data. The simulator writes a complete session with indicative (not real) sensor values, so every tool can be tried without the sensor:

```
python3 -m sensorperf.cli.simulate --out demo_session --quick
python3 -m sensorperf.cli.check_captures --session demo_session
python3 -m sensorperf.cli.analyze --session demo_session
```

C.7 Run the tools. Every tool is run as `python3 -m sensorperf.cli.<tool>` followed by its options, as in §4, §5, §11, §12, and §14. With the environment active (C.4) this works from any folder, so it is simplest to keep a working folder for the session, for example `C:\perf\session_2026_10_14`, put `registration.json`, the pose log, and the session folder in it, open the terminal there (C.3), and give file names relative to it. Each tool prints what it wrote. Three things to know:

- A message starting with `ERROR:` means the tool stopped and wrote nothing; it says what is wrong in the input and which pose or file it concerns. Fix that and run it again.
- A message starting with `WARNING:` means the tool finished but something should be looked at.
- `python3 -m sensorperf.cli.<tool> --help` prints the option list (appendix B).

C.8 If something goes wrong.

- "No module named sensorperf": the environment is not active (the prompt does not start with `(.venv)`), or step C.4 was done in a different folder. Activate it and retry; if that fails, redo C.3 and C.4.
- "python3 is not recognized" on Windows: type `python` instead; if that also fails, redo C.1.
- "pip install" fails with a network or certificate error: the computer's internet access is blocked; ask the engineer for the package files or for the proxy settings.
- A traceback (many lines ending in an exception name) from any tool is a software fault, not an input fault: save the whole output and the input files and send them to the engineer.

## Appendix D. Figure index

| Figure | Caption | File |
|---|---|---|
| 1 | Procedure order and data dependencies. Shaded boxes are captures and open boxes are analyses. Registration poses feed every analysis, the noise level from A sets the thresholds of B, C and D, the edge spread function from B predicts the area bias measured in C, and E reuses the B and C frames. The quick-look count on C data sets the D levels (dashed). | `figures/fig_procedure_flow.png` |
| 2 | The experimental setup, side view. The sensor stands on its own rigid stand, separate from the robot. The robot carries the target on the dowel-pinned adapter anywhere between Z_MIN and Z_MAX. The dial indicator on its fixed stand reads the adapter's Z move in series Z. The enclosure holds the ambient IR constant. | `figures/fig_setup.png` |
| 3 | Front views of T2, T3a, T3b, T4-S, T4-L, T5-S and T5-L to one scale, drawn from the code's own target definitions with the indicative sensor geometry. The real layout depends on the final focal length (unconfirmed). Gray is the back plate, dashed circles are blank sites, vermillion dots are post-only control sites. | `figures/fig_targets.png` |
| 4 | Chamfered cutout and disk on its post, cross-section, not to scale. The three viewpoints (left camera, projector, right camera) pass the knife edge in open space and never meet the beveled wall. The bevel angle is measured from the plate normal. | `figures/fig_chamfer.png` |
| 5 | Left: the 11 noise stations of A, the 5 shape stations of B, C and D, and the 3 reduced stations of the slow tests, along Z. Right: the 5 field positions (codes 0 to 4) in the image, and the square span of the random lateral offsets (phase jitter) at one corner, true size and magnified. | `figures/fig_stations.png` |
| 6 | The default full plan: target centers in the sensor frame, side view (H against Z) and front view (H against V), colored by series, with the frustum. Each cloud around a station is a set of random lateral offsets. | `figures/fig_plan.png` |
| 7 | The series Z visits at one Z0. Left: the step ladder, the 8 step sizes in turn, each with its A, B, A, B alternation. Right: the fine staircase. A circle marks the dial-indicator reading logged at every visit. | `figures/fig_zstep.png` |
| 8 | The session folder feeds the six analyses, which write into `analysis/`. The fits of A and the boundary terms of E converge on `forward_model_parameters.json`. | `figures/fig_data_flow.png` |

## Appendix E. Open questions

These questions of the specification decide how the test is run. Answer them before the session; the answers fill the dagger values, the chamfer check, and the time budget.

- [ ] VSX3000 f_x, f_y, baseline, projector offset, field of view, depth LSB, and frame rate in the chosen trigger mode. These fill the dagger parameters, the chamfer check (§2), and the time budget (§13).
- [ ] Can the emitter be switched off, or a flood illuminator switched on, for fiducial imaging? This decides between the independent and the depth-only registration (§4).
- [ ] Which on-sensor processing configuration is the production one (§3, step 2)?
- [ ] Which surface finish represents the production targets, and is a second finish needed (§2)?
- [ ] Does the robot controller report the actual pose at the time of capture (§1, §5)?
- [ ] Is a dial indicator or capacitive probe with 1 micrometer resolution or better available for series Z (§8)?
