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
| 6-axis robot | Repeatability 0.05 mm or better (ISO 9283); reports the actual, encoder-derived pose at 0.01 mm resolution, time-stamped against the frames; approach-from-below moves programmable; payload above the heaviest target plus adapter | Commanded, repeatable target poses; the read-back pose is the step truth of series Z |
| Quick-change target adapter | Dowel-pinned, re-mount repeatability 0.02 mm or better; every target carries the same dowel datum | Targets can be swapped without re-registering; the datum places their features |
| Temperature loggers (sensor housing and air) | One sample every 1 minute | For drift attribution |
| Capture computer | VSX3000 SDK, the LRVisionLibs `MatCloud` reader, and a robot interface that logs the actual pose read back from the robot | Synchronized capture and logging |

Table 1. Equipment, requirements, and the reason for each.

The robot must report its actual (encoder-derived) pose at the time of the capture, not only the commanded one, to a resolution of 0.01 mm, time-stamped against the frames. If it cannot, tell the engineer before you start: for series Z the read-back pose is the step truth. The laboratory is enclosed and its lighting is constant, so there is no light meter and no ambient-light log; keep the lighting unchanged for the whole session (§3).

## 2. Targets

Every target uses a two-plane construction: a front surface with knife edges, standing a known gap G in front of a back plate of the same finish. One geometry therefore serves the edge, area, detection, and boundary-bias tests. The two gaps are 15 mm (the small gap) and 60 mm (the large gap), set with spacers. Fabrication tolerances are left to the fabricator. Instead, every feature is measured as built and logged (the as-built record, below). Table 2 lists the targets and Figure 3 shows them to one scale.

| ID | Target | Construction | Used in |
|---|---|---|---|
| T2 | Noise and registration plate | Uniform matte, 400, 400 mm (width, height), flat to 0.05 mm, no pattern. Registration uses its plane (§4). | Registration, A, Z, sentinels |
| T3a | Raised square | Square of side 160 mm with knife edges, on hidden posts at the gap G above a back plate. Spacers set G to 15 or 60 mm. | B edges, E |
| T3b | Square window | Front plate with a square window of the same size and knife edges. The back plate is at G behind it. | B edges, E |
| T4 | Disk plate | Back-beveled disks on thin posts at G above a back plate: 3 disks on the feature ladder, 3 blank sites, and 1 post-only control site. | C, D, E |
| T5 | Cutout plate | Back-beveled holes in a front plate, with the back plate at G, removable for the open-background variant (§9): 3 holes on the feature ladder and 3 blank sites. | C, D, E |

Table 2. The targets (specification Section 3.2).

![](figures/fig_targets.png)

Figure 3. Front views of T2, T3a, T3b, T4 and T5 to one scale, drawn from the code's own target definitions with the indicative sensor geometry. The real layout depends on the final focal length (unconfirmed). Gray is the back plate, dashed circles are blank sites, vermillion dots are post-only control sites.

**Feature ladder.** The sensor responds to the subtended size D_px = D f_x / Z, not to the diameter in millimeters. The robot sweeps each feature through the ladder stations from Z_MIN to Z_MAX, a factor of 4 in distance, so one feature covers two octaves of D_px. The smallest feature is 3 px at the far station, and each next feature is larger by a factor of 2 sqrt(2), so neighbors overlap by half an octave; the overlap is the scaling test of the analyses (§14). With the indicative f_x of 688 px (unconfirmed) the 3 features per plate are 7.0, 19.7 and 55.8 mm, covering 3 to 12, 8.5 to 34 and 24 to 96 px over the working range. The lower end sits below the minimum detectable size expected for the VSX3000 (the specification's estimate is 10 to 15 px), so the series reaches the point where detection disappears. Features on a plate are spaced at least 30 px apart, edge to edge, evaluated at Z_MAX (70 mm at the indicative f_x), so the sensor's spatial interpolation cannot couple neighboring features at any station. The plan tool's layout is the source of the as-built drawing, and each plate must fit the field of view at Z_MIN.

**Two planes.** In the disk plate and T3a the front material is the disks (or the square) and the back plate is seen around them. In the cutout plate and T3b the front material is a plate and the back plate is seen through the holes (or the window). Spacers set the gap. Measure the real gap and record it.

**Blank and control sites.** Each plate has 3 blank sites, one for each feature: blank site i is a patch of plain surface sized to the search window of feature i at Z_MAX (the feature plus a margin of 3 px on each side). These give the false-alarm rate in series D at every station. The disk plate also has 1 post-only site (a post with no disk). It shows whether the support post itself is detected.

**Datum.** Every target mounts on the same dowel datum, and the as-built record (below) includes each feature's offset from that datum. Registration from the plane of T2 does not observe where a target sits sideways on the flange, so the once-per-mount check (Step 4.8) locates each mounted target against the left IR image and compares it with those offsets.

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

**Disk support posts.** The post must sit behind the disk and be thinner than 0.5 times the expected D_0, which the specification puts at about 7 px (4 mm at Z_MIN for f_x of 688 px). Posts of 2 mm or less pass, such as hypodermic tubing or wire. The post-only control site and the post check of §10 confirm the post is not detected.

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

**Step 4.1. Environment.** The laboratory is enclosed and its lighting is constant; keep it unchanged for the whole session. Start the temperature loggers (sensor housing and air, one sample every 1 minute) and log the temperatures in `environment_log.csv`.

**Step 4.2. Sensor configuration (Engineer).** Disable auto-exposure and fix exposure, gain, emitter power, and trigger mode. Record every depth-processing setting (temporal filter, spatial filter, hole filling, confidence threshold) in `sensor_config.json`, with its SDK name and value. Record the SDK version and the sensor firmware version in the same file (in its `notes` entry, as in the example below). Characterize the configuration that production will use. If the filters can be switched off, the engineer decides whether to run series A and series B a second time with the filters off, so the sensor's own processing can be separated from its physics (§6, step 8; §7, step 6). This filters-off repeat is outside the capture budget of §13. Give the configuration a short identifier (`config_id`); it goes into every manifest row.

**Step 4.3. Warm-up.** Power the sensor for at least 45 minutes. Then put T2 fronto-parallel at Z = 800 mm (`Z_REFERENCE_MM`) and capture 10 frames every 1 minute. The engineer computes the mean plane Z of each capture. Here sigma_t is the temporal standard deviation of those warm-up frames themselves (not a value taken from another capture or from the datasheet). Start testing once the mean plane Z has drifted less than 0.1 times sigma_t over 10 minutes.

**Optional separate drift run (before the session; the engineer decides).** Do it on the same day or the day before the session, with the robot idle. Put T2 on a fixed stand at the reference station, Z = 800 mm, fronto-parallel and centered in front of the sensor. Power the sensor from cold. Capture 30 frames (`SENTINEL_FRAMES`) every 2 minutes (`DRIFT_RUN_CAPTURE_INTERVAL_MIN`) for 480 minutes (8 hours; `DRIFT_RUN_DURATION_MIN`), starting with a capture at time zero: 241 captures in all. That covers the warm-up and the planned length of the session. Keep the temperature loggers of Step 4.1 running (sensor housing and air). Do not touch the sensor, the stand, or the plate until the run is over. The run measures the sensor's own drift against its temperature; the sentinels in the session stay required, because only they see the robot and the mounts. It costs no robot time and is outside the capture budget (§13). Name the files by the rule of §5: procedure `S`, target `T2`, gap `G0`, station `Z0800`, field `F0`, and a pose number that counts the captures from `P4000` (four digits, so no file shares a name with an in-session sentinel), for example `S_T2_G0_Z0800_F0_P4000_f00.mc` for the first frame and `S_T2_G0_Z0800_F0_P4240_f29.mc` for the last frame of the last capture. Put them in the `sentinels/` folder (§13). Log each frame in the pose log (§11) with its capture time (`timestamp`) and the sensor temperature (`sensor_temp_c`; log the air temperature, `air_temp_c`, too, as for any capture). These two are all the pose log needs for this run: leave the robot pose columns empty, because the plate stands still on its stand and the robot does not hold it. When the run is over, tell the engineer when it started and whether anything interrupted it (a power loss, a missed capture, a touched stand or plate). The engineer plans the run with `plan_stations --drift-run` (§5; it lists the 241 captures in `plan_summary.txt` outside the budget, and `--series` without letters plans the run alone), then builds the manifest with `make_manifest` as for the session (§11), giving that `poses.csv` as `--plan`. `make_manifest` marks the rows `fixed_stand=true` (sub-series `drift_run`), fills the robot pose columns from the nominal pose of the plan, and stops with a message naming the column if the capture time or the sensor temperature is missing. Analysis A uses the run (§14).

**Step 4.4. Settle and vibration check.** With T2 at Z_MIN (400 mm), not at Z_MAX, capture 100 frames twice: once with the servos on, after a move and the settle wait, and once with the brakes engaged. The depth noise is smallest at Z_MIN, so a robot vibration of a given amplitude is easiest to see there. If the servo-on sigma_t exceeds the brakes-on sigma_t by more than 0.1 times the brakes-on value, raise the settle time (now 2 s) or stiffen the target mount, then repeat. Whatever settle time passes this check is the one the robot program uses in §5.

**Step 4.5. Intrinsics and frame checks (Engineer).**

- Read the left IR intrinsics, the depth-to-IR extrinsics, the stereo baseline, the depth LSB, and the projector offset from the SDK or datasheet. These are the values marked with a dagger in appendix A. Fill them in and write them into `sensor_config.json` under `geometry` (the focal lengths and principal point in pixels, the image size, the baseline and projector offset in mm, the depth LSB in mm, and the frame rate). Until this is done the planner and the code use indicative values.
- Confirm that the sensor returns valid depth on T2 at Z_MIN (400 mm) and at Z_MAX (1600 mm). The range limits carry the dagger: if the sensor does not read at an end, tell the engineer, who tightens the limit and rebuilds the station ladder (§5).
- Confirm the depth image is registered to the left IR image. With T3a in view, overlay the square's edges from the IR image on the depth discontinuities. They must agree within 0.5 px.
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

**Step 4.6. Registration capture.** Mount T2 on the dowel-pinned adapter. Take the registration rows of the plan: 30 poses (procedure letter `R`) that span Z_MIN (400 mm) to Z_MAX (1600 mm) and cover the field of view, with tilts about H and about V, of both signs, up to plus or minus 20 degrees (`REGISTRATION_TILT_RANGE_DEG`), so that the plate normals span all three directions. The plate must stay inside the field of view at every tilt (T2 must be fully visible). Registration comes first, so the robot cannot yet be commanded in sensor coordinates: jog the robot by hand to each pose, using the live depth image to reach about the planned depth, field position, and tilt. The exact pose is solved afterward, so hand-jogged poses are fine. At each pose, capture 10 depth frames and the read-back robot pose. Name the files as in §5. Leave the emitter on and the sensor configuration unchanged.

**Step 4.7. Registration solve and acceptance.** Engineer: fit the plate plane in the mean depth frame of each pose (its normal and distance in the camera frame) and solve the hand-eye problem from the plane correspondences: a closed-form start, then joint nonlinear least squares on the plane-normal and plane-distance residuals. The solve gives the camera-to-robot-base transform and the plate's normal and offset on the flange. The plate's sideways position on the flange and its rotation about its normal are not observable from planes, and they are not needed for T2. Write one row per pose into an observations file: `pose_id`, the read-back flange pose (`x_mm`, `y_mm`, `z_mm`, `rotation_type`, `r1` to `r9`, as in §11), and the fitted plane (`nx`, `ny`, `nz`, `distance_mm`). Then run:

```
python3 -m sensorperf.cli.register --observations observations.csv --out registration.json
```

The plane form (`--method planes`) is the default. The tool prints the residual and the verdict.

Accept the registration only if both of these hold:

- The RMS plane-distance residual is 0.15 mm or less (`REGISTRATION_RESIDUAL_ACCEPT_MM`). If it is more, the tool still writes `registration.json` but marks it not accepted, and its exit code is 1.
- The standard error of the camera's Z offset, from the covariance of the fit, is reported next to the residual. About 0.1 mm is expected (Table 8). The register tool prints the residual only, so the engineer takes this standard error from the fit covariance and records it with the result.

If either is too large, add poses with larger tilts, up to plus or minus 20 degrees, or check the mount, then solve again. Save both transforms, the residual, and the standard error of the Z offset in `registration.json` in the session folder (the engineer adds the standard error to the file if the tool did not write it). Note the residual and the standard error in `session_log.md`.

**Step 4.8. Station targets and mount check.** For every row of the plan, the robot pose that puts the target's reference point at the required (H, V, Z) in the camera frame, fronto-parallel unless the row says otherwise, comes from the registration. The planner computes it (§5). Re-mounting a target on the dowel-pinned adapter needs no new registration, provided the mount check passes. Once per mount of any target, at Z = 800 mm, fit the mounted target's front plane from the depth data and compare it with the registered pose:

- Z within 0.15 mm (`REGISTRATION_RESIDUAL_ACCEPT_MM`);
- tilt within 0.05 degrees (`MOUNT_TILT_TOLERANCE_DEG`, a dagger parameter: 0.05 degrees moves a plate edge 200 mm from the center by about 0.17 mm);
- for a target with features (T3a, T3b, T4, T5), also locate one feature edge or outline in the left IR image and compare its H and V position with the as-built datum offsets (§2): both within 0.5 px (`FRAME_CHECK_PX`).

Otherwise re-seat the target or correct the datum record. This is what makes re-mounting on the dowel-pinned adapter safe without a new registration. Log the result in `session_log.md`.

## 5. The plan

Run the planner once the registration is accepted and the sensor values are in `sensor_config.json`. It writes the commanded pose of every capture, in the order to do them.

```
python3 -m sensorperf.cli.plan_stations --out plan/ --registration registration.json \
    --sensor-config sensor_config.json --seed 1
```

Write the seed in `session_log.md`. The same seed and inputs give the same plan again. Without `--registration` the planner writes the target poses in the camera frame only; without `--sensor-config` it uses the indicative sensor geometry and says so loudly, because the field-of-view fit, the lateral offsets in millimeters, the expected depth quantum, and the capture budget (§13) all depend on the real values. Appendix B lists every option.

The engineer decides which optional captures to plan: the filters-off repeat (`--filters-off`, §3 Step 4.2), the staircase (`--staircase`, §8), the lateral sweep (`--lateral-sweep`, §7), the open-background variant (`--open-background`, §9), and the reuse of C frames as D trials (`--reuse-c-first-frames`, §10). All are off by default and, except the reuse, add rows to the pose list. Decide before the first capture and plan once. Adding `--filters-off`, `--open-background`, or `--reuse-c-first-frames` to a later run changes the random offsets of later series, so its pose list cannot be mixed with the first. `--staircase` and `--lateral-sweep` only add rows, so the engineer can plan them later with the same seed and inputs; capture the added rows and keep working from the first list for everything else.

The planner writes five files in the output folder:

- `poses.csv`: one row per commanded pose.
- `plan_summary.txt`: poses and frames per series and station, the capture budget, every adjustment, and every warning. Read it before you start.
- `plan.png`: the planned target centers, a picture of the same kind as Figure 6.
- `targets.json`: the target definitions the plan was made with.
- `parameters.json`: the parameter values the plan was made with (appendix A).

**Terms used in every series.** Four terms are used from here to §10, and each has one meaning.

- Station: a commanded distance of the target center from the sensor, along the optical axis, in mm (the target's Z). The stations are listed below.
- Field position: where the target center sits in the image. Code 0 is the center. Codes 1 to 4 are the four corners, at 0.6 of the half field (`FIELD_OFFSET_FRACTION`): 1 toward (-H, -V), 2 toward (+H, -V), 3 toward (+H, +V), and 4 toward (-H, +V). Figure 5 shows them. A corner is pulled inward when the target would not fit there (the coverage rule below).
- Pose: one commanded target position and orientation. The registration (§4) turns it into the flange pose the robot is sent to. One pose is one move, one settle wait, and the capture of its frames.
- Pose list: the planner's `poses.csv`, one row per pose, in the order to do them (`order`). The robot program executes it row by row. §6 to §10 say what the list holds for each series. You do not choose stations, positions, or order yourself: you follow the list.

One geometric ladder of Z stations serves the whole procedure: 9 stations of the ladder at a ratio of 2^(1/4) (four per octave), 400, 476, 566, 673, 800, 951, 1131, 1345, 1600 mm. Series A, C, and D visit all of them; A also adds the 2 legacy depths 700 and 1000 mm, at the center field position only, so that the existing metrics can be computed at the same depths as the existing data. Series B (edges) visits every second station, the 5 shape stations (400, 566, 800, 1131, 1600 mm). The step ladder of series Z and the tilt sub-series of A visit every fourth station, the 3 reduced stations (400, 800, 1600 mm); the ramp of series Z visits every station of the ladder. The extended trials of D run at the 3 farthest stations (1131, 1345 and 1600 mm). The reference station, 800 mm (a station of the ladder), serves the warm-up check, the sentinels, the re-mount check, the field sub-series of C, the open-background variant, and the post check of D. The working range of 400 to 1600 mm carries the dagger of appendix A, and Step 4.5 confirms it.

The stations the plan visits, and the five field positions in the image, are in Figure 5. The whole plan is in Figure 6.

![](figures/fig_stations.png)

Figure 5. Left: the 9 stations of the ladder, used by A, C and D and by the ramp of Z (A adds the 2 legacy depths, at the center only), the 5 shape stations of B, the 3 reduced stations of the Z step ladder and the A tilt sub-series, and the 3 farthest stations of the extended D trials, along Z; the dotted line is the reference station. Right: the 5 field positions (codes 0 to 4) in the image, and the square span of the random lateral offsets (phase jitter) at one corner, true size and magnified.

![](figures/fig_plan.png)

Figure 6. The default full plan: target centers in the sensor frame, side view (H against Z) and front view (H against V), colored by series, with the frustum. Each cloud around a station is a set of random lateral offsets.

**What a row of `poses.csv` contains.** Table 4 lists the procedure letters. A row has the order to execute it in (`order`), the identity that names the files (`procedure`, `target_id`, `gap_mm`, `station_z_mm`, `field`, `pose_index`), the number of frames (`frames`), a sub-series label (`subseries`: main, tilt, remount, filters_off, sentinel, drift_run, nominal, jitter, ladder, ramp, staircase, lateral_sweep, field, open, extended), the logged random seed (`seed`), the random lateral offset in mm at the station depth (`offset_h_mm`, `offset_v_mm`), the tilt (`tilt_axis`, `tilt_deg`), the commanded Z step and visit (`step_mm`, `visit`, series Z only), the feature index (`level_index`, usually empty, because one frame sees every feature of a plate), and the wanted target pose in the camera frame (`target_x_mm` to `target_rz_deg`: position in mm and a rotation vector in degrees). With a registration the row also holds the flange pose to command in the robot base frame: position and rotation vector (`base_x_mm` to `base_rz_deg`), the rotation matrix (`r00` to `r22`, row by row), and the quaternion (`quat_w`, `quat_x`, `quat_y`, `quat_z`). Use whichever form your robot program accepts. A last column (`notes`) holds extra detail as JSON: for example the achieved field fraction (`field_fraction_achieved`), the approach of a series Z pose (`approach`), the approach side of a lateral-sweep pose (`approach_direction`), and whether a sentinel is the reference of its mount (`mount_reference`).

| Letter | Series | Target | See |
|---|---|---|---|
| R | Registration | T2 | §4 |
| A | Noise plate | T2 | §6 |
| B | Edges | T3a, T3b | §7 |
| Z | Depth steps (B-Z) | T2 | §8 |
| C | Disk and cutout areas | T4, T5 | §9 |
| D | Detection trials | T4, T5 | §10 |
| S | Drift sentinels | whichever target is mounted | §5 |

Table 4. Procedure letters of the plan and of the file names.

**Coverage rule.** The planner checks that each target fits the field of view at its station and field position, with a margin of 8 px (`BOUNDARY_BAND_HALF_WIDTH_PX`) between the plate and the image border (and half the phase-jitter span more for the series with random offsets). A target that would not fit, for example T2 at a corner at Z_MIN, is pulled inward along its field direction until it fits. The fraction of the requested offset that it kept is the achieved field fraction: 1 means no pull-in, and 0 means the target stays at the center. The fraction is in the `notes` of the row (`field_fraction_achieved`) and in the manifest (§11), and `plan_summary.txt` lists every adjustment and every target that does not fit even when centered. Do not override these. Tell the engineer about any plate that does not fit when centered. Keep `plan_summary.txt` with the session: the achieved fractions are reported with the results of series A (per station) and with the field comparison of series C (§14). The disk and cutout plates T4 and T5 are built to fit the field of view at Z_MIN with room for the random offsets; if the target set cannot be built that way, the planner stops with an error that names the plate, and the engineer changes the target design before anything is fabricated.

**Robot program outline**, for each row of `poses.csv`, in `order`:

1. Move to the pose. Use a joint move to a point short of it along the target normal, then a linear move onto it, so the approach is the same every time. For every pose of series Z (step ladder, ramp, and the optional staircase) the point short of the pose lies below it: back off by 2 mm toward smaller Z, then move up onto the pose, so that every pose of the series is approached from below and backlash does not enter the difference between visits (§8). The one exception is the optional lateral sweep of series B (§7), where the approach side alternates on purpose and the row says which side.
2. Wait the settle time of Step 4.4 (2 s unless the check raised it).
3. Trigger the capture of `frames` frames. Name the files `<proc>_<target>_G<gap>_Z<zzzz>_F<field>_P<pose>_f<frame>.mc` from the row: for example `C_T5_G15_Z0800_F0_P017_f03.mc` is procedure C (area), target T5, gap 15 mm, station Z = 800 mm, field position 0 (center), pose 17, frame 3. The pose number has three digits (`P017`); the filters-off repeat, which is planned outside the budget, numbers its poses from `P1000` and so has four. A target without a back plate (T2) writes `G0`. The frame number has two digits and runs from `f00`.
4. Read the robot's actual reported (encoder-derived) flange pose, not the commanded one, and append it to the pose log (§11). Add the sensor and air temperature and a timestamp.
5. Move on.

**Drift sentinels.** A sentinel (letter `S`) is a capture of whatever target is mounted at that point of the pose list, centered at the reference station, Z = 800 mm (`Z_REFERENCE_MM`), fronto-parallel, with 30 frames (`SENTINEL_FRAMES`). The planner puts sentinel rows in the list before the first pose of series A, every 60 minutes by its estimate of the clock (`DRIFT_SENTINEL_INTERVAL_MIN`), and after the last pose of each series. The row names the target that is mounted at that point (`target_id`, `gap_mm`). The sentinel after the last pose of a series is captured on the target mounted at that moment, so no target is swapped for a sentinel and none is re-mounted for one. T2 serves wherever it is mounted anyway: the sentinels before and after series A and after series Z are on T2. The first sentinel after each mount is that target's reference, and the analysis measures the drift of that target against it (the row's notes say `mount_reference`). Do each sentinel when its row comes up, even if the real time differs from the planner's estimate; the timestamp in the pose log is what the analysis uses. Name the files from the row and keep them in the `sentinels/` folder (§13). The optional separate drift run (§3, Step 4.3) also uses letter `S` and the `sentinels/` folder, but it is not part of this plan: its pose numbers start at `P4000` and its sub-series is `drift_run`.

Do the series in the order of the plan: A, then B, then Z, then C, then D. Do not move the sensor between them. Tell the engineer at once if a series stops early; the later series depend on the earlier ones (Figure 1).

## 6. Series A: the noise plate

This series captures the 9 stations of the ladder at 5 field positions each, and the 2 legacy depths (700 and 1000 mm) at the center only, 100 frames per pose, plus a tilt sub-series. The order of the poses is randomized, with a logged seed, so that slow drift cannot masquerade as a Z dependence. The planner has already done this: follow the `order` column.

1. Mount T2. Do the mount check (Step 4.8). The stations are every station of the ladder from Z_MIN (400 mm) to Z_MAX (1600 mm), 400, 476, 566, 673, 800, 951, 1131, 1345, 1600 mm, each at the five field positions: the center and four corners at 0.6 of the half field, all fronto-parallel (Figure 5). The legacy depths 700 and 1000 mm are at the center only, because the existing metrics use the center.
2. The plate must cover the analysis region at every pose. Where a corner would carry part of the plate out of the image, the planner has pulled the pose inward by the coverage rule (§5) and recorded the achieved field fraction in the row. This happens at the near stations. Do not move such a pose back out. The results of series A report the fraction for each station.
3. Before the first station, the pose list has a drift sentinel on T2 (center, Z = 800 mm, 30 frames). More follow every 60 minutes. The one after the last station is on T2 as well, because T2 is still mounted (§5, drift sentinels).
4. At each station: move, wait the settle time, then capture the frames of the row. Log the read-back robot pose, the sensor temperature, and the timestamps. The air temperature comes from the loggers of Step 4.1.
5. Tilt sub-series. At the center of the reduced stations (400, 800, 1600 mm), T2 is tilted about V, then about H, through each angle of 0, 15, 30, 45 degrees. Capture 50 frames per pose. Incidence angle changes both the per-pixel noise and the fill rate. These rows have sub-series `tilt`. A tilt is planned only where the tilted plate's near edge stays at or beyond Z_MIN: the near edge is at Z minus half the plate width times the sine of the tilt. The planner checks this, leaves out the poses that fail, and lists them under "Skipped poses" in `plan_summary.txt`. With the 400, 400 mm plate (width, height) that rules out every tilt at 400 mm, so the tilt sub-series runs at 800 and 1600 mm. Do not add tilts at the skipped stations by hand.
6. Repeat-mount check. Dismount T2, re-mount it, and repeat the Z = 800 mm center station (sub-series `remount`). The difference shows how much of the bias comes from re-mounting the target; the adapter repeats to 0.02 mm.
7. Do not delete any frames. Tell the engineer if a pose shows a warning in §12.
8. Engineer: if Step 4.2 calls for it, repeat steps 1 to 6 with the sensor's filters off. This repeat is outside the capture budget of §13. Plan it with `--filters-off` and record the new configuration in `sensor_config.json` with a new `config_id`.

## 7. Series B: the edge targets

The edge series gives lateral (H, V) resolution and most of the boundary-bias data. Both edge polarities are measured: T3a (front material inside the square) and T3b (back plate inside the window). At each station the target is moved by small random lateral offsets so that every sub-pixel phase of each edge is sampled.

1. Mount T3a (raised square) with the gap at 15 mm. Mount it square to the image: the slant of 5 degrees is part of the square (the as-built record gives it as `rotation_deg`), so all four edges are slanted relative to the pixel grid. The two near-vertical edges measure H resolution and the two near-horizontal edges measure V. Left and right edges have opposite occlusion geometry relative to the baseline, and so do top and bottom. Do the mount check (Step 4.8).
2. At each of the 5 shape stations (400, 566, 800, 1131, 1600 mm), centered and fronto-parallel: move, settle, and capture 30 frames at the nominal pose (sub-series `nominal`).
3. At the same Z, capture 25 further poses, each with 30 frames (sub-series `jitter`). Each adds a logged random lateral offset, uniform over plus or minus half of the phase-jitter span (8 px at the station Z) in both H and V. The plan holds the offsets in millimeters (`offset_h_mm`, `offset_v_mm`), so the robot only has to execute the row.
4. Repeat steps 2 and 3 with the gap at 60 mm. Comparing the two step heights tests whether the normalized edge response depends on step height. If it does, the depth pipeline is nonlinear and resolution must be quoted together with its step height.
5. Repeat steps 1 to 4 with T3b (square window). The window has the opposite edge polarity: front plate outside, back plate inside.
6. Lateral sweep (optional; the engineer decides, §5). It maps the reported edge position against the true one more finely than the random offsets do. Plan it with `--lateral-sweep`. The pose list then holds 20 poses in H and then 20 in V of T3a with the gap at 15 mm, at the reference station (800 mm), with 30 frames each (sub-series `lateral_sweep`). Each pose moves the square sideways from its nominal pose by one more step of 0.1 px (`LATERAL_SWEEP_STEP_PX`), which is about 0.12 mm at 800 mm, up to 2 px (`LATERAL_SWEEP_SPAN_PX`); the offsets are in `offset_h_mm` and `offset_v_mm`. The rows come right after the other captures of T3a at that gap, so do them while T3a is still mounted, without re-mounting. Here the approach direction changes from pose to pose, on purpose: the first, third, and every odd-numbered pose of an axis is approached from the negative side (-H or -V), and the even-numbered poses from the positive side (+H or +V). The robot's lateral play then shows up in the data. The `notes` column (`approach_direction`) names the side for each row, so program the robot to arrive from that side. This is the only place where the approach is not the same every time (§5, step 1 of the robot program). The sweep is outside the capture budget of §13.
7. Engineer: if Step 4.2 calls for it, repeat series B with the sensor's filters off, as for series A (§6, step 8). This repeat is outside the capture budget of §13.

Follow the order of the plan, which keeps each target mounted for as long as it can. The drift sentinel after the last pose of series B is captured on the target mounted at that moment; do not swap targets for it (§5).

## 8. Series Z: depth steps

The depth series gives the smallest Z step the sensor detects and the size of the quantum in which it reports depth. It uses the noise plate T2 in two captures: a step ladder of small Z moves, and a ramp, in which the plate is tilted by a small angle. This is the series most limited by the robot's repeatability, because the truth of each step is the read-back robot pose, carried into the sensor frame through the registration. The true step between two visits is the difference of their registered front-plane depths along the optical axis (the z component of the target pose, which is the front-plane depth for the fronto-parallel plate of the ladder). Because it is a difference of two registered poses, the registration's translation cancels and its rotation error enters only through the cosine of the angle error, which is negligible: only the robot's relative motion accuracy matters.

Design change: the specification of 2026-10-04 used a dial indicator on the target adapter as the step truth; this procedure uses the read-back robot pose instead, so the smallest rungs carry the robot's repeatability as their uncertainty, and the 0.02 and 0.05 mm rungs of that specification's ladder were removed because their truth would be no better than the robot.

![](figures/fig_zstep.png)

Figure 7. The series Z captures at one station, 800 mm. Left: the step ladder, the 6 step sizes in turn, each with its A, B, A, B alternation. Right: the ramp, with the depth the sensor reports if it quantizes at the expected quantum (an illustration, not a measurement).

1. Mount T2 centered and fronto-parallel and do the mount check (Step 4.8). The step ladder visits the 3 reduced stations (400, 800, 1600 mm); the ramp visits all 9 stations of the ladder. Every pose of the series is approached from below (§5, step 1 of the robot program). The drift sentinel after the last pose of the series is on T2, which is still mounted (§5).
2. Step ladder. The rungs are multiples of the expected depth quantum at the station: 0.25, 0.5, 1, 2, 4 and 8 times it (`Z_STEP_LADDER_QUANTA`). The expected quantum grows with the square of Z, so the rungs do too: at the indicative geometry they run from 0.1 to 3.1 mm at 400 mm, 0.388 to 12.4 mm at 800 mm and 1.55 to 49.6 mm at 1600 mm. The planner lists the rungs of each station in millimeters in `plan_summary.txt`; use those. No rung is smaller than 0.1 mm (`ROBOT_MIN_RESOLVABLE_MOVE_MM`), the smallest Z move the robot is trusted to make; a smaller multiple is raised to it. The expected quantum, listed in `plan_summary.txt`, uses the Tier-A disparity quantum until analysis A has measured the real one. For each of the 6 step sizes, alternate the target between Z0 (visit A) and Z0 plus the step (visit B) for 10 cycles (A, B, A, B, and so on). Capture 10 frames at each visit. The analysis uses the read-back pose of every visit, not the commanded step, as ground truth for the step. Alternating cancels linear drift. Rows have sub-series `ladder`, with `step_mm` and `visit` filled in. At the farthest station the largest rung carries the plate beyond Z_MAX; the planner notes this in `plan_summary.txt`. Capture it as planned and write in `session_log.md` whether the sensor returned valid depth there.
3. Ramp. At every station of the ladder, T2 is centered and tilted about H (the horizontal axis, parallel to the stereo baseline) by a small angle, so that the true depth across the visible height of the plate spans 4 expected quanta (`RAMP_QUANTA`). Each image row then lies at one true depth, and the rows step through the quanta. The angle is in the row (`tilt_deg`) and is 0.3 degrees at the nearest station to 3.6 degrees at the farthest. The flange pose in the row already includes it. Capture 50 frames (`FRAMES_PER_RAMP_POSE`) and log the read-back pose (sub-series `ramp`). One capture per station replaces a staircase of small Z moves and does not depend on the robot resolving them. At 400 mm the tilted plate's near edge would come 1.11 mm closer than Z_MIN, so the planner moves the plate center 1.11 mm farther. The station label stays at that station, and the shift is in the row's `notes` (`ramp_center_shift_mm`) and in `plan_summary.txt`. Do not undo it.
4. No extra capture is needed for the blank windows: the Z0 frames of step 2 serve as the no-step reference for the false-alarm threshold.
5. Staircase (optional second pass). The engineer decides after the ramp has been analyzed. It is run only when the ramp has measured a quantum of at least 0.1 mm, because the staircase step is never smaller than that. The engineer plans it with `--staircase` (§5). At each reduced station T2 is moved from Z0 to Z0 plus 3 expected quanta (`Z_STAIRCASE_QUANTA`), in steps of the larger of one expected quantum divided by 10 (`Z_STAIRCASE_SUBDIVISION`) and 0.1 mm, with 10 frames per step (sub-series `staircase`). It shows what the static ramp cannot: whether the output moves in quantized steps in time at one pixel, and the hysteresis or the temporal filter's response to motion. `plan_summary.txt` lists the step and the number of steps for each station and says where a step was raised to the floor. The horizontal axis of its analysis is the read-back Z of each step, whose uncertainty is the robot repeatability of 0.05 mm; when the step is smaller than that, the analysis notes it. The staircase is outside the capture budget of §13 (75 poses and 750 frames at the indicative geometry).

The smallest rung (0.1 mm) is twice the robot repeatability of 0.05 mm; the ladder starts there because a rung below the repeatability would be captured and flagged rather than measured. Do not try to correct the robot pose by hand. Log the read-back pose as it is.

## 9. Series C: disk and cutout areas

Each plate is captured at many random sub-pixel offsets and at every station of the ladder. Sensed area depends on where a feature's edge falls relative to the pixel grid and the projector dots, and on the subtended size D_px, which the Z sweep varies by a factor of 4 for each feature. The procedure averages over the phase and also measures its spread.

1. The pose list for C holds, for each plate (T4 and T5) and each gap (15 and 60 mm), the 9 stations of the ladder, each with the plate centered and fronto-parallel. The planner randomizes the order within each mounting, so targets are re-mounted as rarely as possible. Mount one plate at a time and do the mount check (Step 4.8).
2. At each station, capture 30 poses, each with 10 frames (sub-series `jitter`). Each has a logged random lateral offset, uniform over plus or minus half of the phase-jitter span (8 px) in H and V.
3. Field sub-series. At Z = 800 mm and the small gap, repeat step 2 for each plate at the four corners, 10 poses each (sub-series `field`). The plates may not fit at the full field offset there, so the planner pulls them inward by the coverage rule (§5) and logs the achieved field fraction in each row. Do not move them back out. The fraction is reported with the field comparison of C (§14).
4. Open-background variant (cutouts, optional). At Z = 800 mm, remove the back plate so that nothing lies within the sensor's range behind the holes. That needs 800 mm of clear space behind the plate (Z_MAX minus the reference station), or a surface beyond the sensor's range. Check the space before you start. Then repeat step 2 (sub-series `open`). This separates the sensor's fill-in behavior from reads of a real back surface. The engineer plans it with `--open-background` (§5); it is outside the capture budget of §13.
5. Drift sentinels appear in the pose list on the plate that is mounted, the one after the last pose of series C included (§5). Capture them where they come; no re-mount is needed.

The post check of §10 uses the first frame of each C pose at Z = 800 mm, so keep those frames. The first frame of each C pose can also count as a D trial (§10, step 5); a later frame of a C pose never does.

## 10. Series D: detection trials

A detection trial is one frame at a fresh random lateral offset. All features and blank sites sit on one plate, so a single frame yields one trial for every feature on that plate at once. There is nothing to choose before the trials: the detection levels are the (feature, station) pairs. the 9 stations of the ladder and the 3 features give 27 values of the subtended size D_px, spaced by the station ratio, with each feature spanning two octaves and overlapping its neighbors by half an octave. The 10 percent and 5 percent levels need many more trials than the 50 percent level, at the levels where detection is rare, which lie at the far stations. The 0 percent level is not measured: the analysis predicts it from the fitted curve, and reports it as a prediction (§14).

1. Post check (Engineer). Run the quick-look detection count on the first frame of each C pose at Z = 800 mm:

```
python3 -m sensorperf.cli.check_captures --session Characterization_20261014/ --pilot 800
```

   The tool prints the detection count at each post-only control site against the false-alarm rate. If any post-only site shows detections above the false-alarm rate, the disk posts must be re-made thinner and the check repeated. The check selects nothing else.
2. Main trials. For each configuration (disk or cutout, either gap, every ladder station), capture 60 poses, each with a new random lateral offset and 1 frame. That gives 60 trials per feature and station. The plan shuffles the pose order with a logged seed. Follow the `order` column. The plan treats each plate separately, because a frame sees only one plate.
3. Extended trials for the 5 percent level. At the 3 farthest stations (1131, 1345 and 1600 mm; `DETECTION_LOW_STATION_COUNT`), where the smallest feature lies near and below the expected threshold, the plan raises the pose count of every configuration to 300 trials (`DETECTION_LOW_TRIALS`; sub-series `extended`, the poses beyond the main trials). Every feature, and every blank site, then has that many trials there. 300 trials measure a detection probability of 5 percent (`DETECTION_LOW_PROBABILITY`) to about plus or minus 2.5 percent at 0.95 confidence, and separate it from the 1 percent false-alarm rate. The 0 percent level is not measured. If time is short, the engineer can lower the number of extended trials: for example 150 trials still measure the 5 percent level, to about plus or minus 3.5 percent, and roughly halve that part of the budget.
4. Independence rule. Never use two frames from the same pose as separate trials. The fixed projector pattern and the target's fixed position make them strongly correlated. Take one frame per pose, always at a new offset. Only the first frame of a C pose may serve as a D trial (step 5); a later frame of a C pose never does. The analysis checks independence after the fact.
5. Reuse (optional). The first frame of each C pose at a matching configuration may count as a D trial: 30 of the 60 main trials of each configuration and station. The engineer asks for it with `--reuse-c-first-frames` (§5). The pose list then holds 30 main D poses instead of 60 for each of the 36 configurations and stations, 1,080 fewer in all (the extended trials are not changed). Capture series C as usual and keep frame `f00` of every C pose. The capture budget (§13) assumes no reuse.

The drift sentinels in series D are captured on the plate mounted at that moment, the one after the last pose of the series included (§5).

## 11. Recording the poses: the pose log and the manifest

The capture software writes the sensor data. The robot pose must be recorded separately. The manifest records the read-back pose, not only the commanded one. The robot program appends one line per pose, or one per frame, to a CSV file, the pose log. Report the flange pose in the robot base frame, in the same frame for the whole session. Do not change the active base frame or tool frame.

Per-pose columns (one row for the frames 0 to `frames` minus 1 of a pose):

```
procedure, target_id, gap_mm, station_z_mm, field, pose_index, frames,
x_mm, y_mm, z_mm, rotation_type, r1, r2, r3, r4, r5, r6, r7, r8, r9
```

Optional columns that end up in the manifest: `timestamp`, `sensor_temp_c`, `air_temp_c` (logged every 1 minute). A one-row-per-frame form also exists: the first column is `file`, the Section 9 file name, followed by `x_mm` and the rest.

- `procedure` to `pose_index`: exactly the values of the plan row, which also name the files (a sentinel row has procedure `S` and the target that is mounted). `gap_mm` is empty for a target without a back plate, which the file name writes as `G0`.
- `x_mm`, `y_mm`, `z_mm`: the reported (actual, encoder-derived) flange position in the robot base frame, in mm, written with at least 2 decimals (a resolution of 0.01 mm). This is a requirement: for series Z the read-back pose is the step truth, and a log rounded to 0.1 mm makes the smallest rungs meaningless. `make_manifest` warns when every `z_mm` of series Z has fewer decimals.
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

Table 6. Manifest columns, in the order the manifest module writes them. Rotations are rotation vectors in degrees.

Further columns may follow the ones in Table 6. They carry extra metadata from the plan; at present the achieved field fraction (`field_fraction_achieved`) for a pose placed at a field position. The analysis of A reads it.

The `subseries` column holds one of these labels: main, tilt, remount, filters_off, sentinel, drift_run, nominal, jitter, ladder, ramp, staircase, lateral_sweep, field, open, extended. The labels `ladder`, `ramp`, and `staircase` belong to series Z, `nominal`, `jitter`, and `lateral_sweep` to series B, `field` and `open` to series C, `extended` and `jitter` to series D, and `main` to the main stations of the other series. The labels `filters_off`, `staircase`, `lateral_sweep`, `open`, and `drift_run` mark the optional captures. The module also defines warmup, settle_servo, settle_brakes, mount_check, pilot for the engineer's own checks (Steps 4.3, 4.4, 4.8 and the post check of §10); the planner never writes them. The pose index of the filters-off repeat starts at 1000, so its file names have four digits (§5). The pose index of the optional drift run (§3, Step 4.3) starts at 4000, and its files are in the sentinels folder.

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

The `sentinels/` folder holds the sentinels of the session and, if it was made, the optional drift run (§3, Step 4.3; sub-series `drift_run`, pose numbers from `P4000`).

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
- [ ] If the drift run was made: its 241 captures in `sentinels/`, and the temperature logs that cover it
- [ ] `session_log.md` with: date, sensor serial number, warm-up time, settle time, base frame name, plate flatness and finish, mount-check results, target changes with times, and anything unusual
- [ ] Photos of the setup: sensor stand, each target on the adapter

**Capture budget.** Table 7 is computed from the default plan. The estimate assumes 10 frames per second and 3 s per move plus settle. Both are assumptions; the VSX3000 frame rate in the chosen trigger mode should replace them. The table has a separate row for the drift sentinels.

| Series | Poses | Frames | Robot time (h) |
|---|---:|---:|---:|
| Registration | 30 | 300 | 0.03 |
| A main + tilt | 64 | 5,600 | 0.21 |
| B-HV edges | 520 | 15,600 | 0.87 |
| B-Z ladder + ramp | 369 | 4,050 | 0.42 |
| C main + field | 1,160 | 11,600 | 1.29 |
| D main | 2,160 | 2,160 | 1.86 |
| D extended (5 % series) | 2,880 | 2,880 | 2.48 |
| Sentinels | 11 | 330 | 0.02 |
| **Total** | **7,194** | **42,520** | **7.2** |

Table 7. Capture budget of the default plan: poses, frames, and robot time per series.

The plan has 7,194 poses and 42,520 frames, about 7.2 hours of robot time. The extended 5 percent series takes about a third of the robot time. Check storage before starting: multiply the size of one `.mc` frame by 42,520 frames.

Outside the budget, and not in Table 7, are the optional captures, which the engineer plans only when needed (§5):

- the filters-off repeat of A, B-HV, and B-Z (Step 4.2): 1,027 poses, 25,900 frames, about 1.6 hours;
- the staircase of series Z (§8, step 5): 75 poses, 750 frames, about 0.08 hours;
- the lateral sweep of series B (§7, step 6): 40 poses, 1,200 frames, about 0.07 hours;
- the open-background variant of series C (§9, step 4): 30 poses, 300 frames, about 0.03 hours;
- the reuse of C frames as D trials (§10, step 5), which takes 1,080 poses out of the D main row instead of adding any;
- the separate drift run (§3, Step 4.3): 241 captures, 7,230 frames over 8 hours, taken before the session with the robot idle, so no robot time.

The figures are for the indicative geometry and the default plan. The budget also leaves out target swaps, warm-up, the mount checks, the registration time (the registration poses are in the table, but not the solve), and the D post check. Allow two to three working days in total.


## 14. Running the analyses

The engineer runs the analyses on the finished session folder. One command runs them all, in the order A, B, Z, C, D, E, because A supplies the noise level that B, C, and D use, and B supplies the edge spread function that C compares with:

```
python3 -m sensorperf.cli.analyze --session Characterization_20261014/ [--only A B Z C D E]
```

`--only` runs a subset (the letters are those of Table 4, with B for the lateral analysis and Z for the depth analysis). An analysis whose frames are absent from the manifest is skipped with a notice. The results go into `analysis/` in the session folder; an optional `--out DIR` changes the folder. The exit code is 0 when every selected analysis ran, 1 when one failed (the others still run), and 2 when the session cannot be read. Each analysis writes a CSV summary, a detail file, and figures (PNG and SVG). The definitions are in the specification, Sections 10 to 14; this section says what each one reports.

**A, noise versus Z** (specification Section 10; series A and the sentinels). Reports the temporal, fixed-pattern, and total depth noise of the plate at each Z (the fixed-pattern noise is the spread of the frame-averaged depth about the registered plane, after the temporal noise that remains in the average is taken out), with the bias, the fill rate, the spatial correlation length, and the depth quantization step, and fits the disparity-noise model. It also reports how noise and fill rate change with incidence angle (the tilt sub-series), measures the drift of every mounted target with the sentinels (each against its own first sentinel) and, where a target drifts by more than 0.1 times sigma_t at the reference station, flags it and corrects the bias of the poses captured on that mount, and computes the existing legacy metrics at 700 and 1000 mm. Writes `A_noise_summary.csv` (one row per station and tilt, with the achieved field fraction of the pose and, for the mount it was captured on, the drift rate `drift_rate_mm_per_h` and the flag `drift_flagged`); `A_sentinel_drift.csv` (one row per mounted target: the drift rate `drift_rate_mm_per_h`, the maximum excursion `max_excursion_mm`, the flag `flagged`, and the correction applied `correction_applied_mm`; when the optional drift run exists, also the predicted drift `predicted_drift_mm` and the attribution `attribution`, which is "sensor" or "robot or mount"); and figures: the noise curves with the model fit, fill rate, noise maps, autocorrelation profiles, depth-code histograms, and the sentinel drift of each mounted target. With the drift run, it also writes `A_drift_run.csv` (one row per capture), `A_drift_run_fit.json` (the line of depth against sensor temperature, and the warm-up time), and a figure of the run (`A_drift_run`). The predicted drift comes from that line and the logged temperature; a difference between predicted and observed drift that is well above the sentinel's own noise points to the robot or a mount, not the sensor. The fitted disparity noise, the noise floor, the quantum, k, and the correlation length go into `forward_model_parameters.json`.

**B-HV, resolution in H and V** (specification Section 11.1; series B). Reports the 10 to 90 percent rise distance and the MTF50 of the depth edge response, for each Z, edge orientation (H or V), polarity, and gap, from two cross-checked estimates (the slanted edge and the robot-stepped poses). It reports whether the result depends on step height, and the edge offset that analysis E uses. It also reports the edge position transfer, which asks how well the position the sensor reports follows the true lateral position of the edge from pose to pose. The columns to read in `B_resolution_summary.csv` are the lateral gain `lateral_gain` (the distance the sensed edge moves divided by the distance the true edge moves; a perfect sensor gives a gain of one) with its standard error, the amplitude of the error that repeats with the pixel period (`pixel_lock_amplitude_px`, pixel locking), and the amplitude of the error that repeats with the spacing of the projector dots (`dot_pitch_amplitude_px`, using the correlation length from A as the spacing, so a dot pitch term is skipped, with a note, when A did not run). The intercept of the same fit is the edge offset. If the optional lateral sweep (§7, step 6) was captured, the file also gives the approach hysteresis `hysteresis_px` (the difference in edge position between poses approached from the negative and from the positive side) and the two periodic amplitudes again, refitted on the sweep; these columns are empty without the sweep. The figure `B_transfer` plots the sensed edge position against the true lateral offset, with the fitted line, for each edge, and the remaining error against the position within a pixel. All of this uses the series B frames as they are and needs no extra capture.

B also writes `B_lsf.csv`, the smoothed line spread function of every edge, one row per edge, Z, orientation, polarity, gap, and signed-distance bin: the distance to the true edge `s_px` and the value `lsf_per_px`, with the number of read pixels in the bin. Analysis C builds its blur kernel from this file, so keep it in `analysis/`: the full run writes it before C starts, and `--only C` reads the copy left by an earlier B run. Writes tables and figures in `analysis/`; confidence intervals come from bootstrapping over poses.

**B-Z, resolution in Z** (specification Section 11.2; series Z). Reports the smallest commanded Z step detected with 50 percent probability for square patches of side 1, 5, 20 px, the step-response gain and its linearity, and the measured quantum from the ramp, and from the staircase where it was captured. For the ramp, the row average of the depth is plotted against the true depth of each row. Where the curve is stepped, the quantum is the width of the plateaus. The curve shows plateaus only where the disparity noise is small compared with the quantum; with noise that is a sizeable part of the quantum the frames dither the quantizer and the averaged curve comes out smooth, so expect plateaus mainly at the far stations. The ramp columns of `Z_resolution_summary.csv` tell you what happened at each station: `ramp_dithered` is set when the row average is smooth although the single pixels are stepped (a finding about the sensor's interpolation or noise, not a failure), and `ramp_quantum_method` says which method gave the quantum, `plateaus` (the plateau widths) or `depth levels` (for a smooth curve, the spacing of the populated depth levels of the pooled ramp readings, the method of analysis A). Compare it with the quantum predicted from A. `Z_ramp_rows.csv` holds the row averages. The step truth is the read-back robot pose through the registration. Rungs whose true step is below the robot repeatability are reported but flagged (`truth_reliable` False), left out of the gain regression, and kept in the detection curve. The smallest detectable step for the large patches may come out as a bound limited by the robot (`delta_50_is_bound`), not a measurement. The robot's own read-back scatter is reported per station. Writes tables and figures in `analysis/`.

**C, true versus sensed area** (specification Section 12; series C). Reports the sensed area of disks and cutouts at the half-height contour compared with the true (as-built) area and with the geometrically visible area, against the diameter in pixels, and the edge bias in mm and px. The geometrically visible area of a cutout, the part of the back plate that the sensor can see through the hole, is given in two versions in `C_area_summary.csv`: the one that needs only the two cameras (`a_geo_cameras_mm2`), and the one that also needs the projector to light the point (`a_geo_projector_mm2`), computed with the projector position recorded in `sensor_config.json` (`PROJECTOR_OFFSET_MM`). With the projector midway between the cameras the two are equal for a round hole; `C_area_details.json` says whether the as-built projector is midway and, where the two differ, which version the data follow better. The 2-D image has its own visible area, `a_geo_2d_mm2`, which treats the 2-D image sensor and its collocated light as a single viewpoint at the position `CAMERA_2D_OFFSET_MM`, recorded in `sensor_config.json` as `camera_2d_offset_mm` (leave it out and the column stays empty). A disk face is fully visible, so for a disk each visible area is the true area. The transfer curves are pooled over the features and stations of the Z sweep, with the feature kept as a factor.

A scaling test compares neighboring features where they overlap in D_px: `C_overlap_test.csv` has one row for each pair of neighboring features, kind, gap, and ratio (sensed over true area, or sensed over visible area), with the mean difference of the two curves over the shared range of D_px, its bootstrap interval, and whether zero lies inside it (`agrees`). Agreement supports D_px as the governing variable; a disagreement is attributed to the Z-dependent noise and reported as the difference between the two curves.

The edge bias b is fitted against Z using only the largest feature of each plate, at the stations where its D_px is at least 14 px. That threshold (`AREA_BIAS_FIT_MIN_D_PX`) is a multiple of the expected minimum detectable diameter, 7 px, so that b does not depend on the diameter; the stations used, and the threshold, are listed per fit in `C_area_details.json`. C then checks b against the edge response of B. It blurs an ideal disk of the as-built diameter with the H and V line spread functions from `B_lsf.csv` (assumed separable) and thresholds it to predict the sensed area, and makes the prediction twice: with the blur alone, and with the edge offset s_50 of B added as growth of the front material, because a blur alone cannot move the edge. Agreement for large diameters and disagreement for small ones means that the sensor's interpolation fills in small features beyond what the edge response explains. It also checks the sign of b: for a disk b should be about minus s_50, and for a cutout about plus s_50. Only when no line spread functions are available (no B result in this run and no `B_lsf.csv` in `analysis/`) does C use a Gaussian blur whose rise distance equals the one B measured (or, without B, the one C measures on its own large features), and it then says so in its notes and in `C_area_details.json`.

For the field variants, each off-axis pose is compared with the on-axis curve and carries the achieved field fraction of that pose (§5) as a factor, so a pose the planner pulled inward is not mistaken for a full off-axis one; the comparison is in the details file. The open-background cutouts are compared at the reference station only: the no-read area against the true area, where fill-in by the front plate shows as a no-read area smaller than the true area. Writes `C_area_summary.csv`, `C_overlap_test.csv` (the scaling test), `C_area_details.json`, and figures: transfer curves with the overlap ranges shaded, edge bias against Z, phase spread against diameter in pixels, and predicted against measured area. The edge bias b is a cross-check of the boundary widths of E.

**D, minimum detectable size** (specification Section 13; series D). Reports the diameter and subtended angle at 50 percent, 10 percent, and 5 percent detection probability (D_50, D_10, and D_5, the last being the lowest point measured), corrected for false alarms, for disks and cutouts, with confidence intervals. It also reports a predicted D_0, labeled as a prediction.

The threshold that decides whether a window holds a feature is set from the blank sites, so that blank windows are called detections at the target false-alarm rate of 1 percent. At each station the analysis pools the blank windows of all the blank sites that are large enough to hold the window of a given feature size, and sets one threshold per station and feature size from that pool. A threshold needs at least 300 blank windows (`DETECTION_MIN_BLANK_WINDOWS`, enough that the quantile rests on several windows above it, not on the single largest); fewer than that and the threshold rests on the few largest values and is poorly determined. Where a station has fewer, the analysis divides each window's statistic by the total depth noise at its station from A and adds the nearest neighboring stations of the same configuration, nearest first, until the count is reached; the threshold is then read from the pooled, noise-scaled values and scaled back with the noise of the station itself. The farthest stations, which carry the extended trials, reach the count on their own. Without A there is no noise level to scale by, so a station with too few blank windows keeps its own and the analysis says so in its notes. In `D_detect_summary.csv` the columns `blank_windows`, `blank_windows_stations`, `tau_pooled`, and `tau_note` give, for each station, the number of blank windows used (the smallest over the feature sizes), the stations whose windows were pooled (the station itself included), whether neighbors were pooled, and the reason where the count is still short. Read them beside `tau_mm`, the false-alarm rate that came out (`gamma`), and its confidence interval (`gamma_lower` and `gamma_upper`): the interval should contain the target, and if it does not, the control is not as good as the threshold suggests. The confidence intervals of D_50, D_10, D_5, and the predicted D_0 are bootstrapped over poses, and each resample sets the thresholds again from its own pooled blank windows.

The psychometric fit is made in the logarithm of D_px, pooled over the (feature, station) pairs; the false-alarm rate comes from the blank sites at each station. The curve is fitted in each of the shapes logistic, cumulative normal, and Weibull. D_50, D_10, and D_5 are all read from the shape that fits best, by deviance, and each comes with the range across the shapes as its model uncertainty (`d50_model_min` and `d50_model_max`, and the same for `d10` and `d5`); a wide range means the shape of the curve, not the data, decides that minimum, which matters most for D_10 and D_5 in the tail. The predicted D_0 is the size at which the fitted curve, extrapolated below D_5, reaches 1 percent (`DETECTION_ZERO_PREDICTION_LEVEL`), converted to millimeters at each station; it is never a measurement, because a smooth curve never reaches zero and no number of trials proves a probability is zero. Before the data are seen, the laser-pencil model of the design expects a minimum of about 7 px (`EXPECTED_D0_PX`). The columns `d50_over_expected` and `d0_predicted_over_expected` divide the fitted D_50 and the predicted D_0 by that expectation, so that a ratio near one means the sensor lands where the prior put it and a ratio well above one means it detects only larger features; the details file states the prior once. The noise at each Z enters as a covariate, and the same scaling test as in C compares neighboring features. Trials that reuse the first frame of a C pose (§10, step 5) are counted and flagged. It also checks the independence of the trials.

Writes these files and the figures. `D_detect_summary.csv` has one row per configuration and station, with every minimum in millimeters, pixels, and milliradians. `D_pooled_summary.csv` has the pooled fit, one row per configuration, with the minimums in pixels, the expected-size ratios, and the noise regression. `D_overlap_test.csv` has the scaling test, one row for each pair of neighboring features, with the kind, gap, field position, and detection rule beside the pair, the difference of the two curves over their shared range, and whether zero lies inside its interval. `D_detect_details.json` has the fitted curves, the per-feature thresholds with the blank windows behind each, the bootstrap, and the notes. The figures show the pooled psychometric curves and the minimum against Z. Only D_50 and D_10 in pixels go into `forward_model_parameters.json`; nothing from C does.

**E, boundary detection bias** (specification Section 14; reuses the B and C frames, no captures of its own). Reports, near a true edge, how often the sensor reports a value where geometry says no stereo read is possible, how often it reports a no-read where a surface was visible, and whether it favors the near or the far surface. The feature-scale profiles are drawn against D_px, pooled over the features and stations of a plate.

*Which pixels may be read.* For every pixel within 8 px of a true edge, the analysis works out from the registered pose whether a depth read is geometrically possible: the left camera and the right camera must both see that point of the surface, and the projector must light it. The projector is taken at the position you recorded in `sensor_config.json` (`PROJECTOR_OFFSET_MM`), not at an assumed midpoint. Every group in `E_boundary_bias.csv` has two rows, one for this rule (the projector rule, the primary one, marked `is_primary_rule`) and one for the check rule, which asks only for the two cameras. When the projector is midway between the cameras the two rules give the same answer for a round or square outline; where they differ, quote the primary rule and mention the check.

*What the rows hold.* Each row gives the read bias `beta_read` (positive when the sensor leans toward filling in values, negative when it leans toward dropping out), the fabricated-read width `w_fab_px` (how far reads extend into the region where no read is possible), the dropout width `w_drop_px` (how far no-reads intrude into the region where a read was possible), and the near-far preference `pi_near` (positive when the sensor favors the near surface). The two widths are in pixels and are easier to use in a simulator than the rates. Each of the four has a percentile interval beside it (`beta_lower` and `beta_upper`, `w_fab_lower_px` and `w_fab_upper_px`, `w_drop_lower_px` and `w_drop_upper_px`, `pi_lower` and `pi_upper`). All four intervals come from the same bootstrap over poses with 2000 resamples (`BOOTSTRAP_RESAMPLES`); the count is recorded in `E_boundary_details.json`. A statistic that cannot be computed in some resamples, for example `pi_near` when no read lands on the wrong side, is left out of its own interval only.

*Do the signs agree?* A sensor that fattens foreground edges claims pixels for the near surface beyond the true edge. That shows up as a positive `pi_near`, a negative edge offset s_50 in B, and an edge bias b from C that is positive for disks and negative for cutouts. `E_boundary_details.json` has an entry `sign_consistency` that lists the expected signs, each measured value that is available (this analysis' own `pi_near` and s_50, and B's edge offset and C's b when those analyses ran in the same run), whether each agrees, and an overall verdict in `direction`. Disagreement is a finding about the sensor or the setup, not a failure of the run; note it in the session log.

Writes `E_boundary_bias.csv`, `E_boundary_details.json`, and figures: outcome profiles against distance to the edge, the bias indices against Z, and the feature-scale outcome fractions against D_px. The four terms `w_fab_px`, `w_drop_px`, `pi_near`, and `beta_read` go into `forward_model_parameters.json` as its boundary terms, taken from the straight-edge frames at the station nearest the reference station, under the primary rule.

`forward_model_parameters.json` is assembled after the selected analyses, from the terms each one measured. It is the hand-off to the Tier-A forward-noise simulator.

## 15. Things that spoil a session

The smallest Z steps and the absolute bias are the measurements most limited by the setup. The relative measurements are robust: noise, rise distance, transfer-curve shape, and detection minimums. Table 8 lists the sources of error. The magnitudes are typical values from the specification, not measured ones; the engineer replaces them with measured values after §4.

| Source | Typical magnitude | Affects | What you do about it |
|---|---|---|---|
| Robot repeatability | 0.02 to 0.05 mm | Smallest Z steps; edge position | The ABAB cycles of the ladder average the scatter; rungs below the repeatability are flagged, and a delta_50 below the smallest reliable rung is reported as a bound; every pose of series Z is approached from below so that backlash cancels; the ramp finds the quantum without small Z moves; the plan averages over phase poses |
| Robot absolute accuracy | 0.2 to 1 mm over large moves | Bias, registration | Registration over many poses spanning the full Z range; tests run as local moves from registered stations |
| Registration residual | Acceptance limit 0.15 mm RMS | Bias, edge offset, small-feature area | Do not skip the acceptance gate; report the residual with every result |
| Plane-registration depth offset | About 0.1 mm (rough estimate: the plane-fit error amplified at the tilt limit, averaged over the poses) | Absolute bias only | Many poses over the full Z range; tilts to the limit of 20 degrees; residual reported with every result |
| As-built diameter uncertainty | Set by the measuring instrument | True area (for example 0.6 percent at 7 mm with 0.02 mm uncertainty) | Measure with the comparator or microscope and record the uncertainty |
| Plate flatness | 0.05 mm | Fixed-pattern noise, bias | Keep the flatness report; map the plate on a CMM if available |
| Thermal drift | Unknown until the sentinels run | Bias, series Z | Warm-up gate, sentinels on the mounted target, the optional separate drift run (§3, Step 4.3), A, B, A, B order, randomized order |
| Front-back interreflection | Unknown | Cutout and small-gap results | Matte finish; both gaps are captured |
| Sensor temporal filter | Depends on the configuration | Noise (underestimated), trial independence | Discard frames after each move until the plane settles (Step 4.4); filters-off repeat if possible |
| Reflectance mismatch | Avoided by design | Edges, area | One finish on all surfaces |

Table 8. Sources of error, their size, and what to do about them.

Things that spoil a session in practice:

- Moving or bumping the sensor. If it happens, everything after it is a new session.
- Changing exposure, gain, any filter, the base frame, or the lighting mid-session.
- Using the commanded pose instead of the reported pose in the log, or rounding the logged position to 0.1 mm: for series Z the read-back pose is the step truth.
- Approaching a pose of series Z (ladder, ramp, or staircase) from above, or from different directions: backlash then enters the difference between visits. The only poses approached from alternating sides are those of the optional lateral sweep (§7, step 6), and only because the row says so.
- Swapping a target to take a drift sentinel: a sentinel is captured on whatever target is mounted, and a swap adds the re-mount error the sentinel is meant to watch.
- Moving a pose that the planner pulled inward back toward its requested field position: the achieved field fraction in the row is what the results report.
- A change in the laboratory lighting during a session (the laboratory is enclosed, with constant lighting, by design).
- Skipping the warm-up or the settle check, or shortening the settle wait.
- Touching the sensor, the stand, or the plate, or cutting the power, during the optional drift run (§3, Step 4.3): the run is only useful as one unbroken record.
- A loose adapter, a spacer that has moved, or a target that has shifted on its dowels: do the mount check (Step 4.8) after every swap.
- Fingerprints, dust, or gloss on a target: wipe with isopropyl alcohol. A shiny spot returns a bright highlight and a bad read.
- Different finish on the front and back plates.
- Using two frames of one pose as two detection trials in series D (§10, step 4).
- Not recording a filter setting or a configuration change in `sensor_config.json`.
- Losing the seed of the plan. Without it the offsets cannot be reproduced; the plan files keep it.

Limitations. Results hold for one surface finish, static targets, mostly fronto-parallel poses, and the sensor configuration recorded in `sensor_config.json`. Registration from planes leaves the camera's depth offset conditioned by the registration tilt range (Table 8), so absolute bias carries that uncertainty; Z-scale and nonlinear bias do not, because a rigid transform cannot absorb them. The range limits of 400 and 1600 mm assume the sensor reads there; Step 4.5 confirms this before any series runs. The 0 percent detection size is a prediction from the fitted curve below the measured 5 percent point, not a measurement. Off-axis results (series A and the field sub-series of C) hold at the field fraction the planner achieved, which can be well below the requested one at the near stations. The smallest detectable Z step for the large patches may come out as a bound set by the robot's repeatability, or by the accuracy with which it reports its pose, not a measurement.

---

## Appendix A. Parameters

Every arbitrary constant in the procedure is a named parameter. The table below is generated from the code (`sensorperf/parameters.py`), so it always shows the values the planner and the analyses use. The values are starting points for the VSX3000 at 400 to 1600 mm.

Values marked with a dagger in the specification depend on VSX3000 datasheet or SDK values that were not available when it was written. They are not in this table: the code leaves them empty until Step 4.5 fills them in `sensor_config.json`, and any computation that needs one stops with a clear message instead of guessing. They are: the left IR focal lengths (`SENSOR_FX_PX`, `SENSOR_FY_PX`) and principal point (`SENSOR_CX_PX`, `SENSOR_CY_PX`), the stereo baseline (`SENSOR_BASELINE_MM`), the projector offset (`PROJECTOR_OFFSET_MM`), the depth LSB (`DEPTH_LSB_MM`), and the frame rate in the chosen trigger mode. The range limits `Z_MIN_MM` and `Z_MAX_MM` are in the table with their suggested values and also carry the dagger: Step 4.5 confirms that the sensor reads at both ends. The tilt tolerance `MOUNT_TILT_TOLERANCE_DEG` of the mount check (Step 4.8) is in the table with its suggested value and also carries the dagger: 0.05 degrees moves a plate edge 200 mm from the center by about 0.17 mm. Where this document quotes a number that depends on the dagger values (the feature diameters, the field fit, the budget), it uses indicative values for a 640 by 480 sensor and says so. These are not datasheet values.

| Name | Value | Meaning |
|---|---|---|
| `Z_MIN_MM` | 400 | Near range limit. † Step 4.5 confirms that the sensor reads at this distance. |
| `Z_MAX_MM` | 1600 | Far range limit. † Step 4.5 confirms that the sensor reads at this distance. |
| `Z_STATION_RATIO` | 1.189 | Ratio of successive stations of the one geometric ladder (four stations per octave); the stations are Z_MIN times this ratio to the power k, rounded to 1 mm, up to and including Z_MAX. |
| `Z_SHAPE_STATION_STRIDE` | 2 | Every this-many-th station is a B-HV shape station (400, 566, 800, 1131, 1600 with the defaults). |
| `Z_REDUCED_STATION_STRIDE` | 4 | Every this-many-th station is a reduced station, used by the slowest tests (B-Z, the A tilt sub-series) (400, 800, 1600 with the defaults). |
| `Z_REFERENCE_MM` | 800 | The reference station (one of the ladder stations) used for sentinels, the warm-up check, the re-mount check, the C field sub-series, the C open-background variant and the D post check. |
| `FIELD_OFFSET_FRACTION` | 0.6 | Off-axis field positions at (+/- f W/2, +/- f H/2) from the center, f of the half field (A, C). |
| `TILT_ANGLES_DEG` | 0, 15, 30, 45 | Plate tilts about H and about V in the A tilt sub-series. A tilt is planned only where the plate's near edge stays at or beyond Z_MIN (``acquisition.plan.tilt_near_edge_mm``); the others are skipped and listed in plan_summary.txt. |
| `WARMUP_MIN_MINUTES` | 45 | Minimum powered time before any capture. |
| `WARMUP_DRIFT_WINDOW_MIN` | 10 | Window over which warm-up drift is judged. |
| `WARMUP_DRIFT_FRACTION_OF_SIGMA` | 0.1 | Allowed drift of the mean plane Z over the window, as a fraction of sigma_t at that Z. |
| `WARMUP_CHECK_INTERVAL_MIN` | 1 | Interval of the warm-up captures (Step 4.3: 10 frames every minute). |
| `WARMUP_CHECK_FRAMES` | 10 | Frames per warm-up capture (Step 4.3). |
| `DRIFT_SENTINEL_INTERVAL_MIN` | 60 | Interval between drift sentinel captures. |
| `SENTINEL_FRAMES` | 30 | Frames per drift sentinel (Section 5, Step 3). |
| `DRIFT_RUN_DURATION_MIN` | 480 | Duration of the OPTIONAL separate drift run (Section 4, Step 3), minutes: long enough to cover the warm-up and the planned session length. The run is made before the session (the same day or the day before) with the robot idle, T2 on a fixed stand at the reference station (Z_REFERENCE_MM), fronto-parallel, the sensor powered from cold. Its captures are SENTINEL_FRAMES frames each, files in the sentinels folder with the sub-series label ``drift_run``. It costs no robot time and is outside the Section 9 budget. |
| `DRIFT_RUN_CAPTURE_INTERVAL_MIN` | 2 | Interval between the captures of the optional drift run, minutes (SENTINEL_FRAMES frames per capture). |
| `ROBOT_SETTLE_TIME_S` | 2 | Wait after motion before a capture; Step 4.4 verifies it. |
| `SETTLE_CHECK_FRAMES` | 100 | Frames of each settle-and-vibration capture (Step 4.4). |
| `SETTLE_SIGMA_EXCESS_FRACTION` | 0.1 | Servo-on sigma_t may exceed brakes-on sigma_t by at most this fraction (Step 4.4). |
| `FRAMES_PER_NOISE_STATION` | 100 | Frames per Z, field and tilt pose in A (the tilt sub-series uses frames_per_tilt_pose). |
| `FRAMES_PER_TILT_POSE` | 50 | Frames per pose of the A tilt sub-series (Section 5, Step 5). |
| `FRAMES_PER_EDGE_POSE` | 30 | Frames per edge pose (B-HV). |
| `FRAMES_PER_ZSTEP_POSE` | 10 | Frames per Z-step pose (B-Z ladder visits). |
| `FRAMES_PER_RAMP_POSE` | 50 | Frames per pose of the B-Z ramp sub-series (Section 6.2): the ramp is one pose per station, so it takes more frames than a ladder visit (the same as a tilt pose of A). |
| `FRAMES_PER_AREA_POSE` | 10 | Frames per area pose (C). |
| `FRAMES_PER_DETECTION_TRIAL` | 1 | Frames per detection trial (D): one, giving single-frame detectability. |
| `FRAMES_PER_REGISTRATION_POSE` | 10 | Frames per registration pose (Section 4; the document's REGISTRATION_FRAMES). |
| `MOVE_AND_SETTLE_TIME_S` | 3 | Robot move plus settle time per pose assumed by the capture budget (Section 9). |
| `PHASE_JITTER_SPAN_PX` | 8 | Square span of the random lateral offsets in pixels at the station depth (B, C, D). |
| `PHASE_JITTER_POSES_EDGE` | 25 | Random-offset poses per edge station (B-HV). |
| `PHASE_JITTER_POSES_AREA` | 30 | Random-offset poses per area configuration (C). |
| `FIELD_SUBSERIES_POSES_AREA` | 10 | Random-offset poses per array at each off-axis field position (Section 7, Step 3). |
| `TIER_A_DISPARITY_QUANTUM_PX` | 0.125 | Disparity quantum q (pixels) from which the planner derives the expected depth quantum dZ_q(Z0) = q Z0^2 / k of the B-Z series (ladder rungs, ramp tilt and the optional staircase step). The default is an indicative value, not a datasheet value; once the ramp (or Analysis A) has measured the quantum, override this parameter in the ``plan_stations --parameters`` JSON and re-plan. |
| `Z_STEP_LADDER_QUANTA` | 0.25, 0.5, 1, 2, 4, 8 | Commanded rungs of the B-Z step ladder, as multiples of the expected depth quantum at the station, dZ_q(Z0) = q Z0^2 / k with the Tier-A q until Analysis A has measured one (Section 6.2). Each rung is raised to at least ``robot_min_resolvable_move_mm``. At the indicative geometry the rungs run from 0.1 to 3.1 mm at 400 mm, 0.39 to 12.4 mm at 800 mm and 1.55 to 50 mm at 1600 mm; the planner lists them per station in plan_summary.txt. |
| `ROBOT_MIN_RESOLVABLE_MOVE_MM` | 0.1 | The smallest Z move the robot is trusted to execute (Neil's statement of what the robot can resolve, mm; an independent parameter, not derived from the repeatability). It is the floor of the step ladder (Section 6.2, Step 2): a ladder rung or a staircase step smaller than this is raised to it. The analysis takes the read-back displacement as the truth in any case. The separate truth rule of Analysis B-Z (twice ``robot_repeatability_mm``, see ``TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO``) gives the same 0.1 mm at the defaults, but the two do not follow each other. |
| `ROBOT_REPEATABILITY_MM` | 0.05 | Position repeatability of the robot (ISO 9283), the equipment requirement of Section 3.1. A ladder rung smaller than ``TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO`` (2) times this has a step truth too close to the robot's own scatter, so Analysis B-Z reports it but flags it ``truth_reliable = False`` (Section 11.2, Step 10). |
| `Z_STEP_APPROACH_OVERSHOOT_MM` | 2 | Every visit of series Z is approached from the same direction so that the backlash and compliance of the robot joints (a different elastic and frictional state after a move in the opposite direction) do not enter the A / B difference: the robot first moves this far below the pose (to a smaller Z, nearer the sensor) and then moves up onto the pose, in the direction of increasing Z. |
| `Z_STEP_REPEATS` | 10 | ABAB cycles for each step size. |
| `RAMP_QUANTA` | 4 | The B-Z ramp tilts the plate about H so that the true depth across the plate's VISIBLE height (the smaller of the plate height and the field height at that Z) spans this many expected depth quanta (Section 6.2). |
| `RAMP_MAX_INTERMEDIATE_FRACTION` | 0.5 | Classification of a B-Z ramp curve as stepped or smooth (Section 11.2, Step 14): the curve counts as stepped when fewer than this fraction of its changes over a half-quantum window are intermediate (fall inside ``ramp_intermediate_band``), and as smooth otherwise. Chosen by argument, not from data: a staircase has about 0 of its window changes in the band (a window lies on a plateau or spans one step) and a smooth ramp has all of them, so the midpoint 0.5 separates the two with the same margin on each side. |
| `RAMP_INTERMEDIATE_BAND` | 0.25, 0.75 | Window changes of a B-Z ramp curve, as fractions of the quantum, that count as intermediate (Section 11.2, Step 14): a change over a half-quantum window is about 0 (on a plateau) or about 1 quantum (across a step) for a staircase and about 0.5 quantum for a smooth ramp, so the band is the central half of the interval between 0 and 1. Chosen by argument, not from data. |
| `Z_STAIRCASE_SUBDIVISION` | 10 | OPTIONAL second pass (staircase): fine-sweep points per expected depth quantum; the step is max(dZ_q / this, ``robot_min_resolvable_move_mm``). |
| `Z_STAIRCASE_QUANTA` | 3 | OPTIONAL second pass (staircase): the sweep runs from Z0 to Z0 plus this many expected quanta (Section 6.2). The staircase is planned only when ``plan_stations --staircase`` asks for it and is outside the Section 9 budget. |
| `Z_STAIRCASE_FRAMES` | 10 | OPTIONAL second pass (staircase): frames per step. |
| `LATERAL_SWEEP_STEP_PX` | 0.1 | OPTIONAL second pass of B-HV (lateral sweep, Section 6.1): lateral step of the edge sweep in pixels at the reference station (0.12 mm there). The sweep is planned only when ``plan_stations --lateral-sweep`` asks for it and is outside the Section 9 budget. |
| `LATERAL_SWEEP_SPAN_PX` | 2 | OPTIONAL second pass of B-HV (lateral sweep): span of the sweep in H and in V, pixels at the reference station. The sweep runs from one step to the full span from the nominal position, so it has ``lateral_sweep_span_px / lateral_sweep_step_px`` poses per axis (20 with the defaults); the nominal position itself (offset 0) is the nominal B pose already captured and is not repeated. |
| `ZSTEP_PATCH_SIZES_PX` | 1, 5, 20 | Side lengths of the square patches of the B-Z analysis (1 px, 5 x 5, 20 x 20). |
| `DETECTION_FALSE_ALARM_TARGET` | 0.01 | Target false-alarm rate per window; sets the threshold tau from the blank-site distribution. |
| `DETECTION_MIN_BLANK_WINDOWS` | 300 | The fewest blank windows that set the threshold tau at a station: three per false alarm at the 1 percent target (detection_false_alarm_target), so that the 99 percent quantile of the blank windows rests on about three windows above it, not on the single largest value. Where a station has fewer blank windows than this (a main station has detection_trials_per_level trials per blank site), the window statistic is divided by sigma_tot(Z) and the nearest neighboring stations are pooled until the count is reached (Section 13, Threshold); the extended stations reach it on their own. |
| `DETECTION_MIN_CONNECTED_PX` | 2 | Minimum connected region counted as a detection. |
| `DETECTION_WINDOW_MARGIN_PX` | 3 | Search window radius equals D/2 in pixels plus this margin. |
| `DETECTION_TRIALS_PER_LEVEL` | 60 | Independent trials per (feature, station) pair in the D series. |
| `DETECTION_LOW_TRIALS` | 300 | Trials per (feature, station) pair at the detection_low_station_count farthest stations, where the smallest feature lies near and below the expected threshold. 300 trials measure a detection probability of 5 percent (detection_low_probability) to about +/- 2.5 percent at the confidence level (the extended trials of Section 8). |
| `DETECTION_LOW_STATION_COUNT` | 3 | Number of farthest stations that carry detection_low_trials trials (1131, 1345 and 1600 mm with the defaults). |
| `DETECTION_LOW_PROBABILITY` | 0.05 | The lowest detection probability the trials measure: D_5, the size at which the false-alarm-corrected detection probability is 5 percent. 300 trials (detection_low_trials) resolve it to about +/- 2.5 percent; a lower level (3 percent) would be marginal at that count. The 0 percent point is not measured (see detection_zero_prediction_level). |
| `DETECTION_ZERO_PREDICTION_LEVEL` | 0.01 | The false-alarm-corrected probability level to which the fitted psychometric curve is extrapolated to give the predicted D_0 (Section 13). The result is a PREDICTION from the fitted curve below the lowest measured point (D_5), never a measurement: a smooth curve never reaches zero and no finite number of trials proves a probability is zero. |
| `DETECTION_LAPSE_RATE_MAX` | 0.05 | Upper bound of the lapse rate lambda in the psychometric fit (Section 13, Step 4). |
| `CONFIDENCE_LEVEL` | 0.95 | Level of all confidence intervals and bounds (B, C, D, E). |
| `BOOTSTRAP_RESAMPLES` | 2000 | Resamples for bootstrap confidence intervals (C, D, E). |
| `INDEPENDENCE_SIGMA_MULTIPLE` | 2 | Lag-1 autocorrelation of a detection sequence must lie within this many standard errors (1/sqrt(n)) of zero (Section 13, Step 1: +/- 2/sqrt(n)). |
| `BOUNDARY_BAND_HALF_WIDTH_PX` | 8 | Analysis band on each side of a true edge (E); also the ROI shrink in A, the reference-plane exclusion in B and C, and the edge margin of the capture planner's field-of-view fit (``acquisition.plan``: a target must lie this many pixels inside the image, so the margin used to pull an off-axis pose inward is the same band the analysis shrinks the region of interest by). |
| `AREA_BIAS_FIT_MIN_D_PX` | 14 | Smallest feature diameter, pixels at the station, that enters the fit of the area edge bias b against Z (C, Section 12, Step 10): the fit uses the largest feature at the stations where its D_px is at least this, so that b does not depend on D. The default is twice (AREA_BIAS_FIT_D0_FACTOR) the 7 px expected minimum detectable diameter (``expected_d0_px``), a feature well above the size at which the sensor starts to lose it. The two are tied only through their defaults: if you override ``expected_d0_px`` in the parameters JSON, override this one with it. |
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
| `LEGACY_METRIC_DEPTHS_MM` | 700, 1000 | Depths at which the legacy metrics are computed (Section 10, Step 12); series A adds them as extra noise stations to the ladder, captured at the center field position only. |
| `STATION_MATCH_TOLERANCE_MM` | 0.5 | Two depths closer than this are the same station (the file-name rule rounds a station to 1 mm). |
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
| `FEATURE_LADDER_RATIO` | 2.828 | Diameter ratio of successive disk and cutout features (a half-octave overlap in subtended pixels between neighbors, since one feature covers two octaves of D_px over the Z range). |
| `FEATURE_MIN_PX_AT_Z_MAX` | 3 | Subtended size, in pixels at Z_MAX, of the smallest feature. |
| `FEATURE_COUNT` | 3 | Features per disk plate and per cutout plate. |
| `BLANK_SITES_PER_PLATE` | 3 | Blank sites per plate (the guess rate gamma of the detection fit). Blank site i serves feature i and is sized to that feature's search window at Z_MAX, D_i + 2 DETECTION_WINDOW_MARGIN_PX p(Z_MAX); at most FEATURE_COUNT. |
| `POST_SITES_PER_PLATE` | 1 | Post-only sites per disk plate (the post check: a bare post must not be detected). |
| `FEATURE_ISOLATION_PX` | 30 | Minimum edge-to-edge spacing between features, pixels at Z_MAX (evaluated at the far station so that neighbors stay separated there, where a pixel covers the most millimeters). Applies edge to edge between every pair of sites of a plate, in both directions (along a row and between rows). |
| `FEATURE_PLATE_MARGIN_PX` | 8 | Front-plate margin beyond the outermost sites of a disk or cutout plate, pixels at Z_MAX (equal to the boundary band BOUNDARY_BAND_HALF_WIDTH_PX, so that the analysis band of an outer cutout edge lies on front material at the far station). The plate (sites plus margin) must fit the field of view at Z_MIN with the phase-jitter span and the boundary band on every side; make_standard_target_set raises an error when it does not. |
| `EXPECTED_D0_PX` | 7 | Expected minimum detectable size D_0 in pixels, from the detectability model of Section 3.2 (about 40 pixels of area, a diameter of 7 px), used before fabrication to size the disk support posts. Not a measured value: Analysis D replaces it with the predicted D_0. |
| `POST_DIAMETER_FRACTION_OF_D0` | 0.5 | Rule for the disk support posts: a post must be thinner than this fraction of the expected D_0 (the post check of Section 8, Step 1 confirms that a bare post is not detected). The planned post diameter is post_diameter_fraction_of_d0 x expected_d0_px x p(Z_MIN) (:meth:`post_diameter_mm`: about 2 mm at the indicative geometry). |
| `FRAME_CHECK_PX` | 0.5 | IR-edge to depth-discontinuity agreement required in Step 4.5. |
| `REGISTRATION_POSES` | 30 | Hand-eye poses spanning the volume (Section 4, Step 6). |
| `REGISTRATION_TILT_RANGE_DEG` | 20 | Half range of the registration tilts about H and V (+/-). |
| `REGISTRATION_RESIDUAL_ACCEPT_MM` | 0.15 | Acceptance limit of the registration residual, mm RMS. |
| `MOUNT_CHECK_DEPTH_MM` | 800 | Depth of the once-per-mount plane-fit check (Section 4, Step 8). |
| `MOUNT_TILT_TOLERANCE_DEG` | 0.05 | Largest tilt difference between a mounted target's fitted front plane and the registered pose that the mount check of Step 4.8 accepts (dagger: 0.05 degrees moves a plate edge 200 mm from center by about 0.17 mm). Z is held to registration_residual_accept_mm and H, V to frame_check_px. |
| `ADAPTER_REMOUNT_REPEATABILITY_MM` | 0.02 | Repeatability of the target adapter when a target is removed and mounted again (the re-mount check of A). |
| `TEMPERATURE_LOG_INTERVAL_MIN` | 1 | Interval at which the sensor and air temperatures are logged during every capture. |

Table 9. The parameters of the procedure, with the values of the default plan.

## Appendix B. Software reference

The code that supports this procedure is the Python package `sensorperf/` in this repository. The command-line tools each import other modules of the package, so they cannot be copied out on their own. Ship the whole `sensorperf/` directory together with `pyproject.toml`, `requirements.txt`, and `tests/`, either as a clone of the repository or as a copy of those items with the directory layout kept.

The tools: `plan_stations` (§5), `register` (§4), `make_manifest` (§11), `check_captures` (§10 and §12), `simulate` (a synthetic session for practice, appendix C), and `analyze` (§14). Table 10 lists the package files with their line counts and what they do. The design document `docs/design/code_design.md` describes the modules.

| File | Lines | What it does |
|---|---:|---|
| `sensorperf/__init__.py` | 16 | sensorperf: code for the VSX3000 Resolution, Area-Fidelity, Detectability, and Noise Characterization Procedure. |
| `sensorperf/acquisition/__init__.py` | 1 | sensorperf.acquisition: see the package docstring and docs/design/code_design.md. |
| `sensorperf/acquisition/check.py` | 526 | Quick-look check of a capture set (Part I of the procedure) and the D pilot post check (Section 8, Step 1; Section 13, Step 2). |
| `sensorperf/acquisition/plan.py` | 1960 | Station and pose planning for Part I of the procedure (Sections 4 to 9). |
| `sensorperf/acquisition/pose_log.py` | 625 | Robot pose log -> capture manifest (document, Section 9). |
| `sensorperf/analysis/__init__.py` | 1 | sensorperf.analysis: see the package docstring and docs/design/code_design.md. |
| `sensorperf/analysis/area.py` | 1594 | Analysis C: true versus sensed area (procedure document, Section 12), Steps 1 to 13. |
| `sensorperf/analysis/boundary.py` | 984 | Analysis E: boundary detection bias (procedure document, Section 14), Steps 1 to 8. |
| `sensorperf/analysis/common.py` | 407 | What every analysis of Part II shares: the registered geometry of a pose (which pixel should see which surface, where the true edges are), the reference planes, |
| `sensorperf/analysis/detection.py` | 1712 | Analysis D: minimum detectable size at 50, 10 and 5 percent, with a PREDICTED 0 percent point (procedure document, Section 13), Steps 1 to 11, in the Z-sweep de |
| `sensorperf/analysis/forward_model.py` | 73 | Assembly of ``forward_model_parameters.json`` (document, Section 1 and Section 10 Step 13, Section 14 Step 8): the hand-off from the analyses to the Tier-A forw |
| `sensorperf/analysis/noise.py` | 1786 | Analysis A: noise versus Z (procedure document, Section 10), on the frames of procedure "A" (the noise-plate series) and the drift sentinels (procedure "S", Ste |
| `sensorperf/analysis/overlap.py` | 268 | The overlap (scaling) test of the Z-sweep design (redesign note, Section 5), shared by Analysis C (area transfer curves) and Analysis D (detection curves). |
| `sensorperf/analysis/resolution_depth.py` | 1303 | Analysis B-Z: effective resolution in depth (procedure document, Section 11.2), from the Z-step series (procedure "Z"): the noise plate T2 at a station Z0, a st |
| `sensorperf/analysis/resolution_lateral.py` | 1304 | Analysis B-HV: effective lateral resolution in H and V (procedure document, Section 11.1), from the edge series (procedure "B"): the raised square T3a and the s |
| `sensorperf/cli/__init__.py` | 1 | sensorperf.cli: see the package docstring and docs/design/code_design.md. |
| `sensorperf/cli/analyze.py` | 98 | Command line: run the Part II analyses on a session folder. |
| `sensorperf/cli/check_captures.py` | 122 | Command line: a quick-look check of a capture session before the long analyses. |
| `sensorperf/cli/make_manifest.py` | 117 | Command line: build the capture manifest (procedure document, Section 9) from the robot's pose log, the capture files and the plan. |
| `sensorperf/cli/plan_stations.py` | 193 | Command line: plan the stations and poses of the characterization capture (procedure document, Sections 4 to 9). |
| `sensorperf/cli/register.py` | 192 | Command line: solve the robot-to-sensor registration (procedure document, Section 4, Steps 6 and 7) from the registration observations. |
| `sensorperf/cli/simulate.py` | 102 | Command line: write a synthetic characterization session. |
| `sensorperf/features/__init__.py` | 1 | sensorperf.features: see the package docstring and docs/design/code_design.md. |
| `sensorperf/features/depth_features.py` | 210 | Per-pixel features computed from depth images: temporal statistics over the frames of one pose, the local surface slopes (s_u, s_v) that are inputs of the corre |
| `sensorperf/features/normals.py` | 97 | Surface normals from a least-squares plane fit over an N x N window of camera-frame points (the downstream 5 x 5 estimator, D-8). |
| `sensorperf/features/planes.py` | 145 | Plane fits and plane-based depth references shared by the analyses. |
| `sensorperf/geometry/__init__.py` | 1 | sensorperf.geometry: see the package docstring and docs/design/code_design.md. |
| `sensorperf/geometry/camera.py` | 89 | Pinhole camera model of the depth sensor, built from a capture file's header. |
| `sensorperf/geometry/registration.py` | 376 | Robot-to-sensor registration (document, Section 4): the two transforms that turn a read-back robot pose into a ground-truth target pose in the camera frame, the |
| `sensorperf/geometry/targets.py` | 793 | The two-plane targets of the characterization procedure (document, Section 3): a front surface with knife-edge features standing a gap G in front of a back plat |
| `sensorperf/geometry/transforms.py` | 95 | Rigid transforms and the rigid fit between two point sets. |
| `sensorperf/io/__init__.py` | 1 | sensorperf.io: see the package docstring and docs/design/code_design.md. |
| `sensorperf/io/capture_set.py` | 113 | Frames of one commanded pose loaded together as a stack (adapted from the calibration repository's capture_set.py for the characterization manifest). |
| `sensorperf/io/manifest.py` | 539 | The capture manifest of the characterization procedure (document, Section 9): one row per captured frame, saying which procedure and target it belongs to, where |
| `sensorperf/io/matcloud.py` | 404 | Reader/writer for Liberty Reach's ".mc" ("Matrix Cloud") file format. |
| `sensorperf/io/qt_datastream.py` | 433 | A minimal reader/writer for Qt5's ``QDataStream`` binary encoding (default stream version, which is what ``MC::toFile``/``MC::fromFile`` use -- see ``MC.cpp`` i |
| `sensorperf/io/session.py` | 143 | The session folder of Section 9 and the small JSON records it holds. |
| `sensorperf/parameters.py` | 689 | Every arbitrary constant of the characterization procedure, as a named parameter (procedure document, Section 2), plus the five geometric relations of that sect |
| `sensorperf/simulate/__init__.py` | 1 | sensorperf.simulate: see the package docstring and docs/design/code_design.md. |
| `sensorperf/simulate/demo_plan.py` | 329 | A small but complete demonstration plan for the synthetic session writer (Sections 4 to 9 of the procedure, drastically reduced), a plausible registration to re |
| `sensorperf/simulate/sensor_model.py` | 446 | INDICATIVE synthetic depth renderer of a :class:`~sensorperf.geometry.targets.TwoPlaneTarget` (design document, Section 5, "simulate/sensor_model.py"). |
| `sensorperf/simulate/session.py` | 177 | Write a synthetic characterization session (design document, Section 5, "simulate/session.py"): render every :class:`~sensorperf.acquisition.plan.PlannedCapture |
| `sensorperf/stats/__init__.py` | 1 | sensorperf.stats: see the package docstring and docs/design/code_design.md. |
| `sensorperf/stats/intervals.py` | 160 | Binomial and bootstrap confidence intervals. |
| `sensorperf/stats/logistic.py` | 184 | Binomial logistic regression with a few covariates, for the noise-covariate model of the detectability analysis (procedure document, Section 13; redesign note,  |
| `sensorperf/stats/psychometric.py` | 560 | Psychometric curves, model-free isotonic thresholds and the threshold (floor) model of the detectability analysis. |

Table 10. The files of the package.

Output of `python3 -m sensorperf.cli.plan_stations --help`:

```
usage: python3 -m sensorperf.cli.plan_stations [-h] --out DIR
                                               [--registration PATH]
                                               [--sensor-config PATH]
                                               [--parameters PATH]
                                               [--series [LETTER ...]]
                                               [--seed N] [--no-extended]
                                               [--filters-off] [--staircase]
                                               [--reuse-c-first-frames]
                                               [--lateral-sweep] [--drift-run]
                                               [--open-background]

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
  --series [LETTER ...]
                        subset of series to plan, by procedure letter: R A B Z
                        C D (R registration, A noise, B edges, Z depth steps,
                        C area, D detection); default all. Without letters
                        (--series alone) no series is planned, which is meant
                        for a plan of the optional drift run alone (--drift-
                        run)
  --seed N              master random seed; every shuffle and offset draws
                        from it and is logged (default 0)
  --no-extended         skip the extended trials of the low point (D_5) of the
                        D series at the farthest stations (Section 8)
  --filters-off         append the filters-off repeat of the A and B series
                        (A, B-HV and B-Z; Section 4, Step 4.2) after each
                        filters-on series; its poses are labeled filters_off
                        and are listed outside the main budget in
                        plan_summary.txt
  --staircase           add the optional second pass of the B-Z series, the
                        fine staircase (Section 6.2), at the reduced stations;
                        its poses are labeled staircase and are listed outside
                        the main budget in plan_summary.txt (the ramp and the
                        step ladder are always planned). To use the disparity
                        quantum measured by the ramp instead of the assumed
                        one (0.125 px), pass it as tier_a_disparity_quantum_px
                        in the --parameters JSON.
  --reuse-c-first-frames
                        let the first frame of each C pose of the same target,
                        gap and station count as a D trial (Section 8, Reuse:
                        30 of the 60 per configuration and station), so that
                        the D main series plans only the remaining poses; the
                        Section 9 budget is still computed without the reuse
                        and plan_summary.txt says how many D poses were taken
                        from C
  --lateral-sweep       add the optional second pass of the B-HV series
                        (Section 6.1, Step 6): the edge target T3a swept in H
                        and then in V at Z_REFERENCE_MM in steps of
                        LATERAL_SWEEP_STEP_PX over LATERAL_SWEEP_SPAN_PX (20
                        poses per axis, approached from alternating
                        directions); its poses are labeled lateral_sweep and
                        are listed outside the main budget in plan_summary.txt
  --drift-run           add the optional separate drift run (Section 4, Step
                        3): T2 on a FIXED STAND at the reference station, the
                        robot idle, SENTINEL_FRAMES frames every
                        DRIFT_RUN_CAPTURE_INTERVAL_MIN minutes for
                        DRIFT_RUN_DURATION_MIN minutes (241 captures with the
                        default parameters; procedure S, sub-series drift_run,
                        pose indices from P4000). The captures are listed
                        outside the main budget in plan_summary.txt (count,
                        frames, pose-index range, file names). Their poses.csv
                        rows hold the nominal pose of T2 at the reference
                        station and the notes field fixed_stand=true:
                        make_manifest copies the robot pose columns from the
                        nominal pose, so the pose log needs only timestamp and
                        sensor_temp_c for them (the registered pose is
                        constant, and the analysis uses only the relative mean
                        Z). Give --series without letters to plan the run
                        alone
  --open-background     add the open-background variant of the C series for
                        the cutout arrays (Section 7, Step 4)
```

Output of `python3 -m sensorperf.cli.register --help`:

```
usage: python3 -m sensorperf.cli.register [-h] --observations CSV
                                          [--method {fiducial,planes}]
                                          [--accept-mm MM] [--out PATH]

Solve the robot-to-sensor registration (camera_to_base, target_to_flange) from
registration observations: the read-back flange poses with depth-plane fits of
the noise plate (the default) or with full target poses (hand-eye). Writes
registration.json and prints the residual and the accept verdict.

options:
  -h, --help            show this help message and exit
  --observations CSV    one row per registration pose: pose_id, x_mm, y_mm,
                        z_mm, rotation_type, r1..r9, and either tx_mm, ty_mm,
                        tz_mm, trx_deg, try_deg, trz_deg (rotation vector in
                        degrees) or nx, ny, nz, distance_mm
  --method {fiducial,planes}
                        planes: from depth-plane fits of the noise plate T2
                        only (the standard, no pattern needed); fiducial:
                        hand-eye from full target poses (default planes)
  --accept-mm MM        acceptance limit of the RMS residual in mm (default:
                        REGISTRATION_RESIDUAL_ACCEPT_MM, 0.15)
  --out PATH            registration file to write (default registration.json)
```

Output of `python3 -m sensorperf.cli.make_manifest --help`:

```
usage: python3 -m sensorperf.cli.make_manifest [-h] --pose-log PATH --captures
                                               DIR --plan PATH --registration
                                               PATH [--sensor-config-id TEXT]
                                               [--out PATH] [--strict]

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
usage: python3 -m sensorperf.cli.check_captures [-h] --session DIR
                                                [--out PATH]
                                                [--min-valid-fraction MIN_VALID_FRACTION]
                                                [--border-margin-px BORDER_MARGIN_PX]
                                                [--plane-residual-warn-mm PLANE_RESIDUAL_WARN_MM]
                                                [--pose-residual-warn-mm POSE_RESIDUAL_WARN_MM]
                                                [--normal-warn-deg NORMAL_WARN_DEG]
                                                [--classification-margin-px CLASSIFICATION_MARGIN_PX]
                                                [--min-plane-pixels MIN_PLANE_PIXELS]
                                                [--pilot Z_MM]
                                                [--pilot-subseries LABEL [LABEL ...]]

Quick-look check of a capture session: valid fraction, border contact and the
front and back plane fits against the registered target, or (with --pilot) the
post check of Section 8, Step 1.

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
  --pilot Z_MM          print the post check (Section 8, Step 1) for the C
                        poses at this station (mm, normally Z_REFERENCE_MM)
                        instead of the check
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
| 1 | Procedure order and data dependencies. Shaded boxes are captures and open boxes are analyses. Registration poses feed every analysis, the noise level from A sets the thresholds of B, C and D, the edge spread function from B predicts the area bias measured in C, and E reuses the B and C frames. The quick-look count on C data checks the disk posts before D (dashed). | `figures/fig_procedure_flow.png` |
| 2 | The experimental setup, side view. The sensor stands on its own rigid stand, separate from the robot. The robot carries the target on the dowel-pinned adapter anywhere between Z_MIN and Z_MAX. The laboratory is enclosed and its lighting is constant. | `figures/fig_setup.png` |
| 3 | Front views of T2, T3a, T3b, T4 and T5 to one scale, drawn from the code's own target definitions with the indicative sensor geometry. The real layout depends on the final focal length (unconfirmed). Gray is the back plate, dashed circles are blank sites, vermillion dots are post-only control sites. | `figures/fig_targets.png` |
| 4 | Chamfered cutout and disk on its post, cross-section, not to scale. The three viewpoints (left camera, projector, right camera) pass the knife edge in open space and never meet the beveled wall. The bevel angle is measured from the plate normal. | `figures/fig_chamfer.png` |
| 5 | Left: the 9 stations of the ladder, used by A, C and D and by the ramp of Z (A adds the 2 legacy depths, at the center only), the 5 shape stations of B, the 3 reduced stations of the Z step ladder and the A tilt sub-series, and the 3 farthest stations of the extended D trials, along Z; the dotted line is the reference station. Right: the 5 field positions (codes 0 to 4) in the image, and the square span of the random lateral offsets (phase jitter) at one corner, true size and magnified. | `figures/fig_stations.png` |
| 6 | The default full plan: target centers in the sensor frame, side view (H against Z) and front view (H against V), colored by series, with the frustum. Each cloud around a station is a set of random lateral offsets. | `figures/fig_plan.png` |
| 7 | The series Z captures at one station, 800 mm. Left: the step ladder, the 6 step sizes in turn, each with its A, B, A, B alternation. Right: the ramp, with the depth the sensor reports if it quantizes at the expected quantum (an illustration, not a measurement). | `figures/fig_zstep.png` |
| 8 | The session folder feeds the six analyses, which write into `analysis/`. The fits of A and the boundary terms of E converge on `forward_model_parameters.json`. | `figures/fig_data_flow.png` |
