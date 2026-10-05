# VSX3000 sensor performance testing: step-by-step procedure

Audience: the robot technician and the test engineer. The technician mounts the targets, programs the robot, and records the captures. No knowledge of the analysis math is needed. Where a step says "run", a computer with Python and this repository is needed (appendix C says how to set it up). The steps that need the engineer are marked "Engineer": filling in the sensor values from the SDK or datasheet (§3), the hand-eye solve and its acceptance (§4), choosing the sensor configuration and any filters-off repeat (§3), the post check for series D (§10), the as-built measurements of the targets (§2), and running the analyses (§14). In this document, "§" followed by a number means that section of this document.

What you are producing: a session folder of sensor capture files (`.mc`), one group of files per robot pose, with a table (the manifest) that says, for every file, which target was in view, where the robot had put it, and what the environment was. The specification calls this the data logging of its Section 9. The analysis software reads the manifest, not the file names. If the manifest is wrong, the results are wrong, so much of this procedure is about getting the table right.

The analysis definitions (what is computed from the captures) are in the characterization procedure specification of 2026-10-04. This document is the step-by-step rendering of its Part I (acquisition) plus a guide to running the analyses of its Part II. It does not repeat the math. "Specification Section N" means that section of the specification.

The order of the five capture series, and what each analysis needs from the others, is in Figure 1. Figure 2 (§1) shows the setup. Figure 3 and Figure 4 (§2) show the targets and their chamfered edges. Figure 5 and Figure 6 (§5) show the stations and the planned poses. Figure 7 (§8) shows the depth-step visits, and Figure 8 (§13) shows how the session folder feeds the analyses.

![](figures/fig_procedure_flow.png)

Figure 1. Procedure order and data dependencies. Shaded boxes are captures and open boxes are analyses. Registration poses feed every analysis, the noise level from A sets the thresholds of B, C and D, the edge spread function from B predicts the area bias measured in C, and E reuses the B and C frames. The quick-look count on C data checks the disk posts before D (dashed).

---

## 1. Equipment

The sensor stays fixed. The robot carries the target, so every target pose is commanded, repeatable, and logged. Figure 2 shows the arrangement. Table 1 lists the equipment and why each item is needed.

![](figures/fig_setup.png)

Figure 2. The experimental setup, side view. The sensor stands on its own rigid stand, separate from the robot. The robot carries the target on the dowel-pinned adapter anywhere between Z_MIN and Z_MAX. The laboratory is enclosed and its lighting is constant.

| Item | Requirement | Why |
|---|---|---|
| VSX3000 sensor on a rigid stand | Stand mechanically separate from the robot base, on an isolated floor or table | Prevents robot motion from moving the sensor |
| 6-axis robot | Repeatability {{VALUE:robot_repeatability_mm}} mm or better (ISO 9283); reports the actual, encoder-derived pose at 0.01 mm resolution, time-stamped against the frames; approach-from-below moves programmable; payload above the heaviest target plus adapter | Commanded, repeatable target poses; the read-back pose is the step truth of series Z |
| Quick-change target adapter | Dowel-pinned, re-mount repeatability {{VALUE:adapter_remount_repeatability_mm}} mm or better; every target carries the same dowel datum | Targets can be swapped without re-registering; the datum places their features |
| Temperature loggers (sensor housing and air) | One sample every {{VALUE:temperature_log_interval_min}} minute | For drift attribution |
| Capture computer | VSX3000 SDK, the LRVisionLibs `MatCloud` reader, and a robot interface that logs the actual pose read back from the robot | Synchronized capture and logging |

Table 1. Equipment, requirements, and the reason for each.

The robot must report its actual (encoder-derived) pose at the time of the capture, not only the commanded one, to a resolution of 0.01 mm, time-stamped against the frames. If it cannot, tell the engineer before you start: for series Z the read-back pose is the step truth. The laboratory is enclosed and its lighting is constant, so there is no light meter and no ambient-light log; keep the lighting unchanged for the whole session (§3).

## 2. Targets

Every target uses a two-plane construction: a front surface with knife edges, standing a known gap G in front of a back plate of the same finish. One geometry therefore serves the edge, area, detection, and boundary-bias tests. The two gaps are {{VALUE:gap_small_mm}} mm (the small gap) and {{VALUE:gap_large_mm}} mm (the large gap), set with spacers. Fabrication tolerances are left to the fabricator. Instead, every feature is measured as built and logged (the as-built record, below). Table 2 lists the targets and Figure 3 shows them to one scale.

| ID | Target | Construction | Used in |
|---|---|---|---|
| T2 | Noise and registration plate | Uniform matte, {{VALUE:noise_plate_size_mm}} mm (width, height), flat to {{VALUE:plate_flatness_mm}} mm, no pattern. Registration uses its plane (§4). | Registration, A, Z, sentinels |
| T3a | Raised square | Square of side {{VALUE:edge_square_size_mm}} mm with knife edges, on hidden posts at the gap G above a back plate. Spacers set G to {{VALUE:gap_small_mm}} or {{VALUE:gap_large_mm}} mm. | B edges, E |
| T3b | Square window | Front plate with a square window of the same size and knife edges. The back plate is at G behind it. | B edges, E |
| T4 | Disk plate | Back-beveled disks on thin posts at G above a back plate: {{DERIVED:feature_count}} disks on the feature ladder, {{DERIVED:blank_site_count}} blank sites, and {{DERIVED:post_site_count}} post-only control site. | C, D, E |
| T5 | Cutout plate | Back-beveled holes in a front plate, with the back plate at G, removable for the open-background variant (§9): {{DERIVED:feature_count}} holes on the feature ladder and {{DERIVED:blank_site_count}} blank sites. | C, D, E |

Table 2. The targets (specification Section 3.2).

![](figures/fig_targets.png)

Figure 3. Front views of T2, T3a, T3b, T4 and T5 to one scale, drawn from the code's own target definitions with the indicative sensor geometry. The real layout depends on the final focal length (unconfirmed). Gray is the back plate, dashed circles are blank sites, vermillion dots are post-only control sites.

**Feature ladder.** The sensor responds to the subtended size D_px = D f_x / Z, not to the diameter in millimeters. The robot sweeps each feature through the ladder stations from Z_MIN to Z_MAX, a factor of 4 in distance, so one feature covers two octaves of D_px. The smallest feature is {{VALUE:feature_min_px_at_z_max}} px at the far station, and each next feature is larger by a factor of {{DERIVED:feature_ratio_text}}, so neighbors overlap by half an octave; the overlap is the scaling test of the analyses (§14). With the indicative f_x of {{DERIVED:indicative_fx_px}} px (unconfirmed) the {{DERIVED:feature_count}} features per plate are {{DERIVED:feature_diameters}}, covering {{DERIVED:feature_px_ranges_text}} over the working range. The lower end sits below the minimum detectable size expected for the VSX3000 (the specification's estimate is 10 to 15 px), so the series reaches the point where detection disappears. Features on a plate are spaced at least {{VALUE:feature_isolation_px}} px apart, edge to edge, evaluated at Z_MAX ({{DERIVED:isolation_mm_at_z_max}} mm at the indicative f_x), so the sensor's spatial interpolation cannot couple neighboring features at any station. The plan tool's layout is the source of the as-built drawing, and each plate must fit the field of view at Z_MIN.

**Two planes.** In the disk plate and T3a the front material is the disks (or the square) and the back plate is seen around them. In the cutout plate and T3b the front material is a plate and the back plate is seen through the holes (or the window). Spacers set the gap. Measure the real gap and record it.

**Blank and control sites.** Each plate has {{DERIVED:blank_site_count}} blank sites, one for each feature: blank site i is a patch of plain surface sized to the search window of feature i at Z_MAX (the feature plus a margin of {{VALUE:detection_window_margin_px}} px on each side). These give the false-alarm rate in series D at every station. The disk plate also has {{DERIVED:post_site_count}} post-only site (a post with no disk). It shows whether the support post itself is detected.

**Datum.** Every target mounts on the same dowel datum, and the as-built record (below) includes each feature's offset from that datum. Registration from the plane of T2 does not observe where a target sits sideways on the flange, so the once-per-mount check (Step 4.8) locates each mounted target against the left IR image and compares it with those offsets.

**Surface finish.** All front and back surfaces use one finish, for example bead-blasted aluminum or a matte coating, with mid-range IR reflectance. Record the finish and, if possible, its reflectance at the projector wavelength. A reflectance difference between plates would bias the edge and area results.

### Chamfered (knife-edge) boundaries

Every boundary that defines an edge, disk, or cutout must be chamfered from the back (Figure 4). Then no camera or projector ray can strike the boundary's side wall, and the sensor sees only the front face and the back plate. An unchamfered wall would be seen as a third surface by one camera and not the other, which corrupts exactly the edge measurements this procedure is meant to make.

![](figures/fig_chamfer.png)

Figure 4. Chamfered cutout and disk on its post, cross-section, not to scale. The three viewpoints (left camera, projector, right camera) pass the knife edge in open space and never meet the beveled wall. The bevel angle is measured from the plate normal.

- Cutouts and the square window: countersink from the back face, so the hole widens away from the sensor.
- Disks and the raised square: bevel the back, so the part narrows away from the sensor (a frustum).
- Land: the flat land left at the front edge must be {{VALUE:edge_land_max_mm}} mm or less.
- Bevel angle: measured from the plate normal, it must exceed the worst-case ray angle plus a margin. The margin is {{VALUE:chamfer_margin_deg}} degrees. The worst-case ray angle is the largest angle between the plate normal and the line from any of the three viewpoints (left camera, right camera, projector) to any edge point, over every pose used.

A rough worked value from the specification, assuming about a 70 degree horizontal field of view and a 50 mm baseline (both unconfirmed): the worst field position at Z_MIN gives about 32 degrees. With the margin that is 42 degrees, so a standard 45 degree bevel (90 degree included countersink) would pass. The engineer recomputes this once the VSX3000 geometry is known. If the result exceeds 45 degrees, use a steeper bevel or reduce the field offset ({{VALUE:field_offset_fraction}} of the half field) for series C, D, and E.

**Disk support posts.** The post must sit behind the disk and be thinner than {{VALUE:post_diameter_fraction_of_d0}} times the expected D_0, which the specification puts at about 7 px (4 mm at Z_MIN for f_x of {{DERIVED:indicative_fx_px}} px). Posts of 2 mm or less pass, such as hypodermic tubing or wire. The post-only control site and the post check of §10 confirm the post is not detected.

### The as-built record

Engineer, with the metrologist: measure each feature's front-face diameter, land width, bevel angle, and plate position with an optical comparator or a calibrated microscope, and its offset from the common dowel datum. Record each value with its measurement uncertainty in `targets_asbuilt.csv`. This is the file the analysis reads. All analyses use these as-built values, never the nominal ones. Table 3 lists the columns. The engineer can write a template with the nominal values for the metrologist to overwrite; an empty numeric cell keeps the nominal value. The datum offsets are the reference of the once-per-mount check (Step 4.8); the position columns of Table 3 are measured from the plate center.

| Column | Meaning |
|---|---|
| `target_id` | Target (T3a, T4, and so on) |
| `site_id` | Name of the site on the target, as in `targets.json` (for example `disk_03`, `blank_03`, `post_00`, `square`) |
| `kind` | Disk, cutout, blank, post, raised square, or square window |
| `x_mm`, `y_mm` | Position of the site center on the front face, from the plate center, in mm |
| `diameter_mm` | Front-face diameter (the side, for a square), in mm |
| `diameter_uncertainty_mm` | Measurement uncertainty of the diameter, in mm |
| `land_mm` | Width of the flat land at the front edge, in mm |
| `bevel_deg` | Bevel angle from the plate normal, in degrees |
| `rotation_deg` | In-plane rotation of a square feature (the slant), in degrees |
| `level_index` | Feature of the ladder, from 0 for the smallest (disks, cutouts, and the blank site that serves the feature); empty otherwise |

Table 3. Columns of `targets_asbuilt.csv`.

## 3. Before anything else

Do these steps in order, once, before the first capture of any series. They fix the conditions of the whole session. Sections 3 and 4 together carry Section 4 of the specification (setup, warm-up, and registration), so their steps are numbered Step 4.1 to Step 4.8 as there, and "Step 4.N" anywhere in this document means those steps.

**Step 4.1. Environment.** The laboratory is enclosed and its lighting is constant; keep it unchanged for the whole session. Start the temperature loggers (sensor housing and air, one sample every {{VALUE:temperature_log_interval_min}} minute) and log the temperatures in `environment_log.csv`.

**Step 4.2. Sensor configuration (Engineer).** Disable auto-exposure and fix exposure, gain, emitter power, and trigger mode. Record every depth-processing setting (temporal filter, spatial filter, hole filling, confidence threshold) in `sensor_config.json`, with its SDK name and value. Record the SDK version and the sensor firmware version in the same file (in its `notes` entry, as in the example below). Characterize the configuration that production will use. If the filters can be switched off, the engineer decides whether to run series A and series B a second time with the filters off, so the sensor's own processing can be separated from its physics (§6, step 8; §7, step 6). This filters-off repeat is outside the capture budget of §13. Give the configuration a short identifier (`config_id`); it goes into every manifest row.

**Step 4.3. Warm-up.** Power the sensor for at least {{VALUE:warmup_min_minutes}} minutes. Then put T2 fronto-parallel at Z = {{VALUE:z_reference_mm}} mm and capture {{VALUE:warmup_check_frames}} frames every {{VALUE:warmup_check_interval_min}} minute. The engineer computes the mean plane Z of each capture. Here sigma_t is the temporal standard deviation of those warm-up frames themselves (not a value taken from another capture or from the datasheet). Start testing once the mean plane Z has drifted less than {{VALUE:warmup_drift_fraction_of_sigma}} times sigma_t over {{VALUE:warmup_drift_window_min}} minutes.

**Step 4.4. Settle and vibration check.** With T2 at Z_MIN ({{VALUE:z_min_mm}} mm), not at Z_MAX, capture {{VALUE:settle_check_frames}} frames twice: once with the servos on, after a move and the settle wait, and once with the brakes engaged. The depth noise is smallest at Z_MIN, so a robot vibration of a given amplitude is easiest to see there. If the servo-on sigma_t exceeds the brakes-on sigma_t by more than {{VALUE:settle_sigma_excess_fraction}} times the brakes-on value, raise the settle time (now {{VALUE:robot_settle_time_s}} s) or stiffen the target mount, then repeat. Whatever settle time passes this check is the one the robot program uses in §5.

**Step 4.5. Intrinsics and frame checks (Engineer).**

- Read the left IR intrinsics, the depth-to-IR extrinsics, the stereo baseline, the depth LSB, and the projector offset from the SDK or datasheet. These are the values marked with a dagger in appendix A. Fill them in and write them into `sensor_config.json` under `geometry` (the focal lengths and principal point in pixels, the image size, the baseline and projector offset in mm, the depth LSB in mm, and the frame rate). Until this is done the planner and the code use indicative values.
- Confirm that the sensor returns valid depth on T2 at Z_MIN ({{VALUE:z_min_mm}} mm) and at Z_MAX ({{VALUE:z_max_mm}} mm). The range limits carry the dagger: if the sensor does not read at an end, tell the engineer, who tightens the limit and rebuilds the station ladder (§5).
- Confirm the depth image is registered to the left IR image. With T3a in view, overlay the square's edges from the IR image on the depth discontinuities. They must agree within {{VALUE:frame_check_px}} px.
- Confirm the baseline direction. The occlusion band (a strip of no-reads) appears beside vertical edges only if the baseline runs along H. If it appears beside horizontal edges, tell the engineer: H and V swap in series B and E. With the right camera at +H, the band lies outside the left edge of the raised square and inside the right edge of the window; if it lies on the other side, tell the engineer: the sign of H is reversed.

Example of `sensor_config.json` (the text in angle brackets is replaced by the real value; the keys are the ones the software reads, so the SDK and firmware versions go into `notes`):

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
  "notes": "SDK <version>; firmware <version>"
}
```

Write the date, sensor serial number, firmware and SDK versions, robot model and controller software version, the active base frame name, the plate flatness report, the finish, and anything unusual in `session_log.md`. Do not change the active base frame, the exposure, or any sensor setting during the session.

## 4. Robot-to-sensor registration

Registration gives every later capture a ground-truth target pose in the sensor frame. Two things anchor it. Robot repeatability fixes relative motion. A plane-correspondence solve on the noise plate T2 fixes the camera's pose in the robot base, using the depth planes themselves: there is no pattern plate, no IR image to evaluate, and no emitter to switch. What the planes cannot observe, the sideways position of a target on the flange, comes from the as-built datum and a once-per-mount check against the left IR image (Step 4.8). After Step 4.7 below, the robot-base-to-sensor transform is known, so every commanded target pose is also a ground-truth pose in sensor coordinates (compare Figure 2).

**Step 4.6. Registration capture.** Mount T2 on the dowel-pinned adapter. Take the registration rows of the plan: {{VALUE:registration_poses}} poses (procedure letter `R`) that span Z_MIN ({{VALUE:z_min_mm}} mm) to Z_MAX ({{VALUE:z_max_mm}} mm) and cover the field of view, with tilts about H and about V, of both signs, up to plus or minus {{VALUE:registration_tilt_range_deg}} degrees (`REGISTRATION_TILT_RANGE_DEG`), so that the plate normals span all three directions. The plate must stay inside the field of view at every tilt (T2 must be fully visible). Registration comes first, so the robot cannot yet be commanded in sensor coordinates: jog the robot by hand to each pose, using the live depth image to reach about the planned depth, field position, and tilt. The exact pose is solved afterward, so hand-jogged poses are fine. At each pose, capture {{VALUE:frames_per_registration_pose}} depth frames and the read-back robot pose. Name the files as in §5. Leave the emitter on and the sensor configuration unchanged.

**Step 4.7. Registration solve and acceptance.** Engineer: fit the plate plane in the mean depth frame of each pose (its normal and distance in the camera frame) and solve the hand-eye problem from the plane correspondences: a closed-form start, then joint nonlinear least squares on the plane-normal and plane-distance residuals. The solve gives the camera-to-robot-base transform and the plate's normal and offset on the flange. The plate's sideways position on the flange and its rotation about its normal are not observable from planes, and they are not needed for T2. Write one row per pose into an observations file: `pose_id`, the read-back flange pose (`x_mm`, `y_mm`, `z_mm`, `rotation_type`, `r1` to `r9`, as in §11), and the fitted plane (`nx`, `ny`, `nz`, `distance_mm`). Then run:

```
python3 -m sensorperf.cli.register --observations observations.csv --out registration.json
```

The plane form (`--method planes`) is the default. The tool prints the residual and the verdict.

Accept the registration only if both of these hold:

- The RMS plane-distance residual is {{VALUE:registration_residual_accept_mm}} mm or less (`REGISTRATION_RESIDUAL_ACCEPT_MM`). If it is more, the tool still writes `registration.json` but marks it not accepted, and its exit code is 1.
- The standard error of the camera's Z offset, from the covariance of the fit, is reported next to the residual. About 0.1 mm is expected (Table 8). The register tool prints the residual only, so the engineer takes this standard error from the fit covariance and records it with the result.

If either is too large, add poses with larger tilts, up to plus or minus {{VALUE:registration_tilt_range_deg}} degrees, or check the mount, then solve again. Save both transforms, the residual, and the standard error of the Z offset in `registration.json` in the session folder (the engineer adds the standard error to the file if the tool did not write it). Note the residual and the standard error in `session_log.md`.

**Step 4.8. Station targets and mount check.** For every row of the plan, the robot pose that puts the target's reference point at the required (H, V, Z) in the camera frame, fronto-parallel unless the row says otherwise, comes from the registration. The planner computes it (§5). Re-mounting a target on the dowel-pinned adapter needs no new registration, provided the mount check passes. Once per mount of any target, at Z = {{VALUE:mount_check_depth_mm}} mm, fit the mounted target's front plane from the depth data and compare it with the registered pose:

- Z within {{VALUE:registration_residual_accept_mm}} mm (`REGISTRATION_RESIDUAL_ACCEPT_MM`);
- tilt within {{VALUE:mount_tilt_tolerance_deg}} degrees (`MOUNT_TILT_TOLERANCE_DEG`, a dagger parameter: {{VALUE:mount_tilt_tolerance_deg}} degrees moves a plate edge 200 mm from the center by about 0.17 mm);
- for a target with features (T3a, T3b, T4, T5), also locate one feature edge or outline in the left IR image and compare its H and V position with the as-built datum offsets (§2): both within {{VALUE:frame_check_px}} px (`FRAME_CHECK_PX`).

Otherwise re-seat the target or correct the datum record. This is what makes re-mounting on the dowel-pinned adapter safe without a new registration. Log the result in `session_log.md`.

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

One geometric ladder of Z stations serves the whole procedure: {{DERIVED:station_count}} stations of the ladder at a ratio of {{DERIVED:station_ratio_text}} (four per octave), {{DERIVED:ladder_text}} mm. Series A, C, and D visit all of them; A also adds the {{DERIVED:legacy_depth_count}} legacy depths {{DERIVED:legacy_depths_text}} mm, so that the existing metrics can be computed at the same depths as the existing data. Series B (edges) visits every second station, the {{DERIVED:shape_station_count}} shape stations ({{DERIVED:shape_stations_text}} mm). Series Z and the tilt sub-series of A visit every fourth station, the {{DERIVED:reduced_station_count}} reduced stations ({{DERIVED:reduced_stations_text}} mm). The extended trials of D run at the {{DERIVED:zero_station_count}} farthest stations ({{DERIVED:zero_stations_text}} mm). The reference station, {{VALUE:z_reference_mm}} mm (a station of the ladder), serves the warm-up check, the sentinels, the re-mount check, the field sub-series of C, the open-background variant, and the post check of D. The working range of {{VALUE:z_min_mm}} to {{VALUE:z_max_mm}} mm carries the dagger of appendix A, and Step 4.5 confirms it.

The stations the plan visits, and the five field positions in the image, are in Figure 5. The whole plan is in Figure 6.

![](figures/fig_stations.png)

Figure 5. Left: the {{DERIVED:station_count}} stations of the ladder, used by A, C and D (A adds the {{DERIVED:legacy_depth_count}} legacy depths), the {{DERIVED:shape_station_count}} shape stations of B, the {{DERIVED:reduced_station_count}} reduced stations of Z and the tilt sub-series, and the {{DERIVED:zero_station_count}} farthest stations of the extended D trials, along Z; the dotted line is the reference station. Right: the {{DERIVED:field_position_count}} field positions (codes 0 to 4) in the image, and the square span of the random lateral offsets (phase jitter) at one corner, true size and magnified.

![](figures/fig_plan.png)

Figure 6. The default full plan: target centers in the sensor frame, side view (H against Z) and front view (H against V), colored by series, with the frustum. Each cloud around a station is a set of random lateral offsets.

**What a row of `poses.csv` contains.** Table 4 lists the procedure letters. A row has the order to execute it in (`order`), the identity that names the files (`procedure`, `target_id`, `gap_mm`, `station_z_mm`, `field`, `pose_index`), the number of frames (`frames`), a sub-series label (`subseries`: for example `main`, `tilt`, `remount`, `nominal`, `jitter`, `ladder`, `staircase`, `field`, `extended`, `sentinel`), the logged random seed (`seed`), the random lateral offset in mm at the station depth (`offset_h_mm`, `offset_v_mm`), the tilt (`tilt_axis`, `tilt_deg`), the commanded Z step and visit (`step_mm`, `visit`, series Z only), the feature index (`level_index`, usually empty, because one frame sees every feature of a plate), and the wanted target pose in the camera frame (`target_x_mm` to `target_rz_deg`: position in mm and a rotation vector in degrees). With a registration the row also holds the flange pose to command in the robot base frame: position and rotation vector (`base_x_mm` to `base_rz_deg`), the rotation matrix (`r00` to `r22`, row by row), and the quaternion (`quat_w`, `quat_x`, `quat_y`, `quat_z`). Use whichever form your robot program accepts. A last column (`notes`) holds extra detail as JSON.

| Letter | Series | Target | See |
|---|---|---|---|
| R | Registration | T2 | §4 |
| A | Noise plate | T2 | §6 |
| B | Edges | T3a, T3b | §7 |
| Z | Depth steps (B-Z) | T2 | §8 |
| C | Disk and cutout areas | T4, T5 | §9 |
| D | Detection trials | T4, T5 | §10 |
| S | Drift sentinels | T2 | §6, step 3 |

Table 4. Procedure letters of the plan and of the file names.

The planner checks that each target fits the field of view at its station. A target that would not fit, for example T2 off-axis at Z_MIN, is pulled inward along its field direction until it fits. `plan_summary.txt` lists every such adjustment and every target that does not fit even when centered. Do not override these. Tell the engineer about any plate that does not fit when centered. The disk and cutout plates T4 and T5 are built to fit the field of view at Z_MIN with room for the random offsets; if the target set cannot be built that way, the planner stops with an error that names the plate, and the engineer changes the target design before anything is fabricated.

**Robot program outline**, for each row of `poses.csv`, in `order`:

1. Move to the pose. Use a joint move to a point short of it along the target normal, then a linear move onto it, so the approach is the same every time. For every visit of series Z the point short of the pose lies below it: back off by {{VALUE:z_step_approach_overshoot_mm}} mm toward smaller Z, then move up onto the pose, so that every visit of the series is approached from below and backlash does not enter the difference between visits (§8).
2. Wait the settle time of Step 4.4 ({{VALUE:robot_settle_time_s}} s unless the check raised it).
3. Trigger the capture of `frames` frames. Name the files `<proc>_<target>_G<gap>_Z<zzzz>_F<field>_P<pose>_f<frame>.mc` from the row: for example `C_T5_G15_Z0800_F0_P017_f03.mc` is procedure C (area), target T5, gap 15 mm, station Z = 800 mm, field position 0 (center), pose 17, frame 3. A target without a back plate (T2) writes `G0`. The frame number runs from `f00`.
4. Read the robot's actual reported (encoder-derived) flange pose, not the commanded one, and append it to the pose log (§11). Add the sensor and air temperature and a timestamp.
5. Move on.

Sentinel rows (letter `S`) appear in the plan every {{VALUE:drift_sentinel_interval_min}} minutes, by the planner's estimate of the clock. A sentinel is T2 at the center at Z = {{VALUE:z_reference_mm}} mm, {{DERIVED:sentinel_frames}} frames. It needs T2 on the robot. If T2 is not mounted when a sentinel row comes up, ask the engineer whether to swap targets now or to do the sentinel at the next target change, and write the actual time of the capture in the pose log. After the sentinel, mount the target of the series again and do the mount check of Step 4.8.

Do the series in the order of the plan: A, then B, then Z, then C, then D. Do not move the sensor between them. Tell the engineer at once if a series stops early; the later series depend on the earlier ones (Figure 1).

## 6. Series A: the noise plate

This series captures the {{DERIVED:station_count}} stations of the ladder plus the {{DERIVED:legacy_depth_count}} legacy depths ({{DERIVED:legacy_depths_text}} mm) at {{DERIVED:field_position_count}} field positions each, {{DERIVED:noise_station_frames}} frames per pose, plus a tilt sub-series. The station order is randomized, with a logged seed, so that slow drift cannot masquerade as a Z dependence. The planner has already done this: follow the `order` column.

1. Mount T2. Do the mount check (Step 4.8). The stations are every station of the ladder from Z_MIN ({{VALUE:z_min_mm}} mm) to Z_MAX ({{VALUE:z_max_mm}} mm), {{DERIVED:ladder_text}} mm, and the legacy depths {{DERIVED:legacy_depths_text}} mm. The five field positions are the center and four corners at {{VALUE:field_offset_fraction}} of the half field, all fronto-parallel (Figure 5).
2. Check that the plate fully covers the analysis region at every station. At Z_MIN off-axis this may limit the field offset. The planner has pulled such poses inward; see `plan_summary.txt`.
3. Before the first station, capture a drift sentinel: center, Z = {{VALUE:z_reference_mm}} mm, {{DERIVED:sentinel_frames}} frames. The plan repeats the sentinel every {{VALUE:drift_sentinel_interval_min}} minutes and after the last station.
4. At each station: move, wait the settle time, then capture the frames of the row. Log the read-back robot pose, the sensor temperature, and the timestamps.
5. Tilt sub-series. At the center and at the {{DERIVED:reduced_station_count}} reduced stations ({{DERIVED:reduced_stations_text}} mm), T2 is tilted about V, then about H, through each angle of {{VALUE:tilt_angles_deg}} degrees. Capture {{VALUE:frames_per_tilt_pose}} frames per pose. Incidence angle changes both the per-pixel noise and the fill rate. These rows have sub-series `tilt`.
6. Repeat-mount check. Dismount T2, re-mount it, and repeat the Z = {{VALUE:z_reference_mm}} mm center station (sub-series `remount`). The difference shows how much of the bias comes from re-mounting the target; the adapter repeats to {{VALUE:adapter_remount_repeatability_mm}} mm.
7. Do not delete any frames. Tell the engineer if a pose shows a warning in §12.
8. Engineer: if Step 4.2 calls for it, repeat the series with the sensor's filters off. This repeat is outside the capture budget of §13. Plan it with `--filters-off` and record the new configuration in `sensor_config.json` with a new `config_id`.

## 7. Series B: the edge targets

The edge series gives lateral (H, V) resolution and most of the boundary-bias data. Both edge polarities are measured: T3a (front material inside the square) and T3b (back plate inside the window). At each station the target is moved by small random lateral offsets so that every sub-pixel phase of each edge is sampled.

1. Mount T3a (raised square) with the gap at {{VALUE:gap_small_mm}} mm. Mount it square to the image: the slant of {{VALUE:edge_slant_deg}} degrees is part of the square (the as-built record gives it as `rotation_deg`), so all four edges are slanted relative to the pixel grid. The two near-vertical edges measure H resolution and the two near-horizontal edges measure V. Left and right edges have opposite occlusion geometry relative to the baseline, and so do top and bottom. Do the mount check (Step 4.8).
2. At each of the {{DERIVED:shape_station_count}} shape stations ({{DERIVED:shape_stations_text}} mm), centered and fronto-parallel: move, settle, and capture {{VALUE:frames_per_edge_pose}} frames at the nominal pose (sub-series `nominal`).
3. At the same Z, capture {{VALUE:phase_jitter_poses_edge}} further poses, each with {{VALUE:frames_per_edge_pose}} frames (sub-series `jitter`). Each adds a logged random lateral offset, uniform over plus or minus half of the phase-jitter span ({{VALUE:phase_jitter_span_px}} px at the station Z) in both H and V. The plan holds the offsets in millimeters (`offset_h_mm`, `offset_v_mm`), so the robot only has to execute the row.
4. Repeat steps 2 and 3 with the gap at {{VALUE:gap_large_mm}} mm. Comparing the two step heights tests whether the normalized edge response depends on step height. If it does, the depth pipeline is nonlinear and resolution must be quoted together with its step height.
5. Repeat steps 1 to 4 with T3b (square window). The window has the opposite edge polarity: front plate outside, back plate inside.

6. Engineer: if Step 4.2 calls for it, repeat series B with the sensor's filters off, as for series A (§6, step 8). This repeat is outside the capture budget of §13.

Follow the order of the plan, which keeps each target mounted for as long as it can.

## 8. Series Z: depth steps

The depth series gives the smallest Z step the sensor detects. It uses small Z moves of the noise plate. This is the series most limited by the robot's repeatability, because the truth of each step is the read-back robot pose, carried into the sensor frame through the registration. The true step between two visits is the difference of their registered front-plane depths along the optical axis (the z component of the target pose, which is the front-plane depth for the fronto-parallel plate of this series). Because it is a difference of two registered poses, the registration's translation cancels and its rotation error enters only through the cosine of the angle error, which is negligible: only the robot's relative motion accuracy matters.

Design change: the specification of 2026-10-04 used a dial indicator on the target adapter as the step truth; this procedure uses the read-back robot pose instead, so the smallest rungs carry the robot's repeatability as their uncertainty, and the 0.02 and 0.05 mm rungs of that specification's ladder were removed because their truth would be no better than the robot.

![](figures/fig_zstep.png)

Figure 7. The series Z visits at one Z0. Left: the step ladder, the {{DERIVED:zstep_rung_count}} step sizes in turn, each with its A, B, A, B alternation. Right: the fine staircase.

1. Mount T2 centered and fronto-parallel at each Z0 of the {{DERIVED:reduced_station_count}} reduced stations ({{DERIVED:reduced_stations_text}} mm). Approach every visit of the series from below (§5, step 1).
2. Step ladder. For each of the {{DERIVED:zstep_rung_count}} step sizes ({{VALUE:z_step_ladder_mm}} mm), alternate the target between Z0 (visit A) and Z0 plus the step (visit B) for {{VALUE:z_step_repeats}} cycles (A, B, A, B, and so on). Capture {{VALUE:frames_per_zstep_pose}} frames at each visit. The analysis uses the read-back pose of every visit, not the commanded step, as ground truth for the step. Alternating cancels linear drift. Rows have sub-series `ladder`, with `step_mm` and `visit` filled in.
3. Fine staircase. Sweep from Z0 to Z0 plus {{VALUE:z_staircase_quanta}} times the expected depth quantum, in steps of one expected quantum divided by {{VALUE:z_staircase_subdivision}}, with {{VALUE:z_staircase_frames}} frames per step (sub-series `staircase`). This reveals whether the output moves in quantized steps, or is smoothed by the sensor's interpolation. The expected quantum, listed in `plan_summary.txt`, uses the Tier-A disparity quantum until analysis A has measured the real one. If the engineer has re-planned the staircase with the measured quantum, use the new rows. The horizontal axis of the staircase is the read-back Z of each step, whose uncertainty is the robot repeatability of {{VALUE:robot_repeatability_mm}} mm; when the fine step is smaller than that, the analysis notes it and the plateau widths carry that uncertainty.
4. No extra capture is needed for the blank windows: the Z0 frames of step 2 serve as the no-step reference for the false-alarm threshold.

The smallest rung ({{DERIVED:zstep_smallest_mm}} mm) is twice the robot repeatability of {{VALUE:robot_repeatability_mm}} mm; the ladder starts there because a rung below the repeatability would be captured and flagged rather than measured. Do not try to correct the robot pose by hand. Log the read-back pose as it is.

## 9. Series C: disk and cutout areas

Each plate is captured at many random sub-pixel offsets and at every station of the ladder. Sensed area depends on where a feature's edge falls relative to the pixel grid and the projector dots, and on the subtended size D_px, which the Z sweep varies by a factor of 4 for each feature. The procedure averages over the phase and also measures its spread.

1. The configuration list is both plates {T4, T5}, with both gaps, at each of the {{DERIVED:station_count}} stations of the ladder. Center the plate and keep it fronto-parallel. The plan randomizes the order within each mounting, so targets are re-mounted as rarely as possible. Mount one plate at a time and do the mount check (Step 4.8).
2. At each configuration, capture {{VALUE:phase_jitter_poses_area}} poses, each with {{VALUE:frames_per_area_pose}} frames (sub-series `jitter`). Each has a logged random lateral offset, uniform over plus or minus half of the phase-jitter span in H and V.
3. Field sub-series. At Z = {{VALUE:z_reference_mm}} mm and the small gap, repeat step 2 for each plate at the four corners, {{VALUE:field_subseries_poses_area}} poses each (sub-series `field`).
4. Open-background variant (cutouts, optional). At Z = {{VALUE:z_reference_mm}} mm, remove the back plate so that nothing lies within the sensor's range behind the holes. Then repeat step 2. This separates the sensor's fill-in behavior from reads of a real back surface. Plan it with `--open-background`.
5. Capture the drift sentinels the plan lists (§5).

The post check of §10 uses the first frame of each C pose at Z = {{VALUE:z_reference_mm}} mm, so keep those frames.

## 10. Series D: detection trials

A detection trial is one frame at a fresh random lateral offset. All features and blank sites sit on one plate, so a single frame yields one trial for every feature on that plate at once. There is nothing to choose before the trials: the detection levels are the (feature, station) pairs. the {{DERIVED:station_count}} stations of the ladder and the {{DERIVED:feature_count}} features give {{DERIVED:level_count}} values of the subtended size D_px, spaced by the station ratio, with each feature spanning two octaves and overlapping its neighbors by half an octave. The 10 percent and 0 percent minimums need many more trials at the levels where detection disappears, which lie at the far stations.

1. Post check (Engineer). Run the quick-look detection count on the first frame of each C pose at Z = {{VALUE:z_reference_mm}} mm:

```
python3 -m sensorperf.cli.check_captures --session Characterization_20261014/ --pilot 800
```

   The tool prints the detection count at each post-only control site against the false-alarm rate. If any post-only site shows detections above the false-alarm rate, the disk posts must be re-made thinner and the check repeated. The check selects nothing else.
2. Main trials. For each configuration (disk or cutout, either gap, every ladder station), capture {{VALUE:detection_trials_per_level}} poses, each with a new random lateral offset and {{VALUE:frames_per_detection_trial}} frame. That gives {{VALUE:detection_trials_per_level}} trials per feature and station. The plan shuffles the pose order with a logged seed. Follow the `order` column. The plan treats each plate separately, because a frame sees only one plate.
3. Extended trials for the 0 percent point. At the {{DERIVED:zero_station_count}} farthest stations ({{DERIVED:zero_stations_text}} mm), where the smallest feature lies below the expected threshold, the plan raises the pose count of every configuration to {{VALUE:detection_zero_trials}} trials (sub-series `extended`, the poses beyond the main trials). Every feature, and every blank site, then has that many trials there. With zero detections in n trials, the one-sided upper bound on detection probability at {{VALUE:confidence_level}} confidence is about 3/n. So {{VALUE:detection_zero_trials}} trials bound a "never detected" level at {{DERIVED:zero_bound_percent}} percent ({{VALUE:detection_zero_probability_bound}}). If time is short, the engineer can lower the number of extended trials: for example 150 trials bound the 0 percent point at 2 percent and roughly halve that part of the budget.
4. Independence rule. Never use two frames from the same pose as separate trials. The fixed projector pattern and the target's fixed position make them strongly correlated. Take one frame per pose, always at a new offset. The analysis checks independence after the fact.
5. Reuse. The first frame of each C pose at a matching configuration may count as a D trial. The plan does not count it, so it does not reduce the number of D poses planned.

## 11. Recording the poses: the pose log and the manifest

The capture software writes the sensor data. The robot pose must be recorded separately. The manifest records the read-back pose, not only the commanded one. The robot program appends one line per pose, or one per frame, to a CSV file, the pose log. Report the flange pose in the robot base frame, in the same frame for the whole session. Do not change the active base frame or tool frame.

Per-pose columns (one row for the frames 0 to `frames` minus 1 of a pose):

```
procedure, target_id, gap_mm, station_z_mm, field, pose_index, frames,
x_mm, y_mm, z_mm, rotation_type, r1, r2, r3, r4, r5, r6, r7, r8, r9
```

Optional columns that end up in the manifest: `timestamp`, `sensor_temp_c`, `air_temp_c` (logged every {{VALUE:temperature_log_interval_min}} minute). A one-row-per-frame form also exists: the first column is `file`, the Section 9 file name, followed by `x_mm` and the rest.

- `procedure` to `pose_index`: exactly the values of the plan row, which also name the files. `gap_mm` is empty for a target without a back plate.
- `x_mm`, `y_mm`, `z_mm`: the reported (actual, encoder-derived) flange position in the robot base frame, in mm, written with at least {{DERIVED:pose_log_min_decimals}} decimals (a resolution of 0.01 mm). This is a requirement: for series Z the read-back pose is the step truth, and a log rounded to 0.1 mm makes the smallest rungs meaningless. `make_manifest` warns when every `z_mm` of series Z has fewer decimals.
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
C,T5,15,800,0,17,10,812.40,-33.21,455.02,euler_zyx_deg,-91.3,19.8,0.4,
Z,T2,,800,0,42,10,812.43,-33.19,455.01,euler_zyx_deg,-91.3,19.8,0.4,
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
| Readings | `timestamp`, `sensor_temp_c`, `air_temp_c`, `sensor_config_id` |
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

The tool also has an option for the post check of series D (§10, step 1): `--pilot 800` prints the detection counts at the post-only sites instead of the check.

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
- [ ] Photos of the setup: sensor stand, each target on the adapter

**Capture budget.** Table 7 is computed from the default plan. The estimate assumes {{DERIVED:budget_frame_rate_hz}} frames per second and {{VALUE:move_and_settle_time_s}} s per move plus settle. Both are assumptions; the VSX3000 frame rate in the chosen trigger mode should replace them. Target swaps, warm-up, the D post check, and the filters-off repeat of A and B (Step 4.2) are excluded. Allow two to three working days in total.

{{BUDGET_TABLE}}

Table 7. Capture budget of the default plan: poses, frames, and robot time per series.

The plan has {{DERIVED:total_poses_text}} poses and {{DERIVED:total_frames_text}} frames, about {{DERIVED:total_robot_hours}} hours of robot time. The extended 0 percent series takes about a third of the robot time. Check storage before starting: multiply the size of one `.mc` frame by {{DERIVED:total_frames_text}} frames.

## 14. Running the analyses

The engineer runs the analyses on the finished session folder. One command runs them all, in the order A, B, Z, C, D, E, because A supplies the noise level that B, C, and D use, and B supplies the edge spread function that C compares with:

```
python3 -m sensorperf.cli.analyze --session Characterization_20261014/ [--only A B Z C D E]
```

`--only` runs a subset (the letters are those of Table 4, with B for the lateral analysis and Z for the depth analysis). An analysis whose frames are absent from the manifest is skipped with a notice. The results go into `analysis/` in the session folder; an optional `--out DIR` changes the folder. The exit code is 0 when every selected analysis ran, 1 when one failed (the others still run), and 2 when the session cannot be read. Each analysis writes a CSV summary, a detail file, and figures (PNG and SVG). The definitions are in the specification, Sections 10 to 14; this section says what each one reports.

**A, noise versus Z** (specification Section 10; series A and the sentinels). Reports the temporal, fixed-pattern, and total depth noise of the plate at each Z, with the bias, the fill rate, the spatial correlation length, and the depth quantization step, and fits the disparity-noise model. It also reports how noise and fill rate change with incidence angle (the tilt sub-series), corrects for drift with the sentinels, and computes the existing legacy metrics at {{DERIVED:legacy_depths_text}} mm. Writes `A_noise_summary.csv` (one row per station) and figures: the noise curves with the model fit, fill rate, noise maps, autocorrelation profiles, and depth-code histograms. The fitted disparity noise, the noise floor, the quantum, k, and the correlation length go into `forward_model_parameters.json`.

**B-HV, resolution in H and V** (specification Section 11.1; series B). Reports the 10 to 90 percent rise distance and the MTF50 of the depth edge response, for each Z, edge orientation (H or V), polarity, and gap, from two cross-checked estimates (the slanted edge and the robot-stepped poses). It reports whether the result depends on step height, and the edge offset that analysis E uses. Writes tables and figures in `analysis/`; confidence intervals come from bootstrapping over poses.

**B-Z, resolution in Z** (specification Section 11.2; series Z). Reports the smallest commanded Z step detected with 50 percent probability for square patches of side {{VALUE:zstep_patch_sizes_px}} px, the step-response gain and its linearity, and the measured quantum from the fine staircase. The step truth is the read-back robot pose through the registration. Rungs whose true step is below the robot repeatability are reported but flagged (`truth_reliable` False), left out of the gain regression, and kept in the detection curve. The smallest detectable step for the large patches may come out as a bound limited by the robot (`delta_50_is_bound`), not a measurement. The robot's own read-back scatter is reported per station. Writes tables and figures in `analysis/`.

**C, true versus sensed area** (specification Section 12; series C). Reports the sensed area of disks and cutouts at the half-height contour compared with the true (as-built) area and with the geometrically visible area, against the diameter in pixels, and the edge bias in mm and px. The transfer curves are pooled over the features and stations of the Z sweep, with the feature kept as a factor. A scaling test compares neighboring features where they overlap in D_px: agreement within the bootstrap interval supports D_px as the governing variable, and a disagreement is attributed to the Z-dependent noise and reported. It also compares with the prediction from the edge response of B, and the field and open-background variants. Writes `C_area_summary.csv`, `C_overlap_test.csv` (the scaling test), and figures: transfer curves, edge bias against Z, and phase spread against diameter in pixels.

**D, minimum detectable size** (specification Section 13; series D). Reports the diameter and subtended angle at 50 percent, 10 percent, and 0 percent detection probability, corrected for false alarms, for disks and cutouts, with confidence intervals. The psychometric fit is made in the logarithm of D_px, pooled over the (feature, station) pairs; the false-alarm rate comes from the blank sites at each station; the 0 percent value is reported as a D_px bracket and converted to millimeters at each station, and it is a bound that is shown, with {{VALUE:confidence_level}} confidence, to be below {{VALUE:detection_zero_probability_bound}}, not a proof of zero. The noise at each Z enters as a covariate, and the same scaling test as in C compares neighboring features. It also checks the independence of the trials. Writes `D_detect_summary.csv` (one row per configuration and station), `D_pooled_summary.csv` (the pooled fit), `D_overlap_test.csv` (the scaling test), and figures: pooled psychometric curves and the minimum against Z.

**E, boundary detection bias** (specification Section 14; reuses the B and C frames, no captures of its own). Reports, near a true edge, how often the sensor reports a value where geometry says no stereo read is possible, how often it reports a no-read where a surface was visible, and whether it favors the near or the far surface. The feature-scale profiles are drawn against D_px, pooled over the features and stations of a plate. Writes `E_boundary_bias.csv` and figures: outcome profiles against distance to the edge, and the bias indices against Z. The widths and the near-far preference go into `forward_model_parameters.json`.

`forward_model_parameters.json` is assembled after the selected analyses, from the terms each one measured. It is the hand-off to the Tier-A forward-noise simulator.

## 15. Things that spoil a session

The smallest Z steps and the absolute bias are the measurements most limited by the setup. The relative measurements are robust: noise, rise distance, transfer-curve shape, and detection minimums. Table 8 lists the sources of error. The magnitudes are typical values from the specification, not measured ones; the engineer replaces them with measured values after §4.

| Source | Typical magnitude | Affects | What you do about it |
|---|---|---|---|
| Robot repeatability | 0.02 to 0.05 mm | Smallest Z steps; edge position | The ABAB cycles of the ladder average the scatter; rungs below the repeatability are flagged, and a delta_50 below the smallest reliable rung is reported as a bound; every visit of series Z is approached from below so that backlash cancels; the plan averages over phase poses |
| Robot absolute accuracy | 0.2 to 1 mm over large moves | Bias, registration | Registration over many poses spanning the full Z range; tests run as local moves from registered stations |
| Registration residual | Acceptance limit {{VALUE:registration_residual_accept_mm}} mm RMS | Bias, edge offset, small-feature area | Do not skip the acceptance gate; report the residual with every result |
| Plane-registration depth offset | About 0.1 mm (rough estimate: the plane-fit error amplified at the tilt limit, averaged over the poses) | Absolute bias only | Many poses over the full Z range; tilts to the limit of {{VALUE:registration_tilt_range_deg}} degrees; residual reported with every result |
| As-built diameter uncertainty | Set by the measuring instrument | True area (for example 0.6 percent at 7 mm with 0.02 mm uncertainty) | Measure with the comparator or microscope and record the uncertainty |
| Plate flatness | {{VALUE:plate_flatness_mm}} mm | Fixed-pattern noise, bias | Keep the flatness report; map the plate on a CMM if available |
| Thermal drift | Unknown until the sentinels run | Bias, series Z | Warm-up gate, sentinels, A, B, A, B order, randomized order |
| Front-back interreflection | Unknown | Cutout and small-gap results | Matte finish; both gaps are captured |
| Sensor temporal filter | Depends on the configuration | Noise (underestimated), trial independence | Discard frames after each move until the plane settles (Step 4.4); filters-off repeat if possible |
| Reflectance mismatch | Avoided by design | Edges, area | One finish on all surfaces |

Table 8. Sources of error, their size, and what to do about them.

Things that spoil a session in practice:

- Moving or bumping the sensor. If it happens, everything after it is a new session.
- Changing exposure, gain, any filter, the base frame, or the lighting mid-session.
- Using the commanded pose instead of the reported pose in the log, or rounding the logged position to 0.1 mm: for series Z the read-back pose is the step truth.
- Approaching a visit of series Z from above, or from different directions: backlash then enters the difference between visits.
- A change in the laboratory lighting during a session (the laboratory is enclosed, with constant lighting, by design).
- Skipping the warm-up or the settle check, or shortening the settle wait.
- A loose adapter, a spacer that has moved, or a target that has shifted on its dowels: do the mount check (Step 4.8) after every swap.
- Fingerprints, dust, or gloss on a target: wipe with isopropyl alcohol. A shiny spot returns a bright highlight and a bad read.
- Different finish on the front and back plates.
- Using two frames of one pose as two detection trials in series D (§10, step 4).
- Not recording a filter setting or a configuration change in `sensor_config.json`.
- Losing the seed of the plan. Without it the offsets cannot be reproduced; the plan files keep it.

Limitations. Results hold for one surface finish, static targets, mostly fronto-parallel poses, and the sensor configuration recorded in `sensor_config.json`. Registration from planes leaves the camera's depth offset conditioned by the registration tilt range (Table 8), so absolute bias carries that uncertainty; Z-scale and nonlinear bias do not, because a rigid transform cannot absorb them. The range limits of {{VALUE:z_min_mm}} and {{VALUE:z_max_mm}} mm assume the sensor reads there; Step 4.5 confirms this before any series runs. A "0 percent" minimum means below the stated bound with {{VALUE:confidence_level}} confidence, not proven zero. The smallest detectable Z step for the large patches may come out as a bound set by the robot's repeatability, or by the accuracy with which it reports its pose, not a measurement.

---

## Appendix A. Parameters

Every arbitrary constant in the procedure is a named parameter. The table below is generated from the code (`sensorperf/parameters.py`), so it always shows the values the planner and the analyses use. The values are starting points for the VSX3000 at {{VALUE:z_min_mm}} to {{VALUE:z_max_mm}} mm.

Values marked with a dagger in the specification depend on VSX3000 datasheet or SDK values that were not available when it was written. They are not in this table: the code leaves them empty until Step 4.5 fills them in `sensor_config.json`, and any computation that needs one stops with a clear message instead of guessing. They are: the left IR focal lengths (`SENSOR_FX_PX`, `SENSOR_FY_PX`) and principal point (`SENSOR_CX_PX`, `SENSOR_CY_PX`), the stereo baseline (`SENSOR_BASELINE_MM`), the projector offset (`PROJECTOR_OFFSET_MM`), the depth LSB (`DEPTH_LSB_MM`), and the frame rate in the chosen trigger mode. The range limits `Z_MIN_MM` and `Z_MAX_MM` are in the table with their suggested values and also carry the dagger: Step 4.5 confirms that the sensor reads at both ends. The tilt tolerance `MOUNT_TILT_TOLERANCE_DEG` of the mount check (Step 4.8) is in the table with its suggested value and also carries the dagger: 0.05 degrees moves a plate edge 200 mm from the center by about 0.17 mm. Where this document quotes a number that depends on the dagger values (the feature diameters, the field fit, the budget), it uses indicative values for a 640 by 480 sensor and says so. These are not datasheet values.

{{PARAMETER_TABLE}}

Table 9. The parameters of the procedure, with the values of the default plan.

## Appendix B. Software reference

The code that supports this procedure is the Python package `sensorperf/` in this repository. The command-line tools each import other modules of the package, so they cannot be copied out on their own. Ship the whole `sensorperf/` directory together with `pyproject.toml`, `requirements.txt`, and `tests/`, either as a clone of the repository or as a copy of those items with the directory layout kept.

The tools: `plan_stations` (§5), `register` (§4), `make_manifest` (§11), `check_captures` (§10 and §12), `simulate` (a synthetic session for practice, appendix C), and `analyze` (§14). Table 10 lists the package files with their line counts and what they do. The design document `docs/design/code_design.md` describes the modules.

{{FILE_TABLE}}

Table 10. The files of the package.

Output of `python3 -m sensorperf.cli.plan_stations --help`:

{{CLI_HELP:plan_stations}}

Output of `python3 -m sensorperf.cli.register --help`:

{{CLI_HELP:register}}

Output of `python3 -m sensorperf.cli.make_manifest --help`:

{{CLI_HELP:make_manifest}}

Output of `python3 -m sensorperf.cli.check_captures --help`:

{{CLI_HELP:check_captures}}

Output of `python3 -m sensorperf.cli.simulate --help`:

{{CLI_HELP:simulate}}

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

{{FIGURE_INDEX}}
