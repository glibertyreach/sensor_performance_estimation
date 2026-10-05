"""
Write a synthetic characterization session (design document, Section 5, "simulate/session.py"):
render every :class:`~sensorperf.acquisition.plan.PlannedCapture` of a plan into the Section 9 folder
layout with its manifest, so that every acquisition tool and analysis can run without the sensor.

What is simulated, per pose
    1. The commanded flange pose is ``registration.flange_to_base_for(capture.target_to_camera)``.
    2. The robot does not reproduce it exactly: the read-back pose is the commanded one plus Gaussian
       translation noise of ``robot_repeatability_mm`` per axis (0 means a perfect robot).
    3. The realized ground-truth target pose is ``registration.target_to_camera(read_back_pose)``.
       THAT pose is rendered and written to the manifest's target pose columns, exactly as in a real
       session where the analyses only know the read-back pose and the registration.
    4. ``max(1, ceil(frames * frame_scale))`` frames are rendered with
       :func:`~sensorperf.simulate.sensor_model.render_frame` and written under
       ``root / SERIES_DIRS[procedure] / capture.file_name(frame)``.

Clock. Timestamps are ISO-8601 from a fixed start (``SESSION_START``). Poses are visited in
``capture.order``; each pose advances the clock by (frames / frame rate + move-and-settle time). The
elapsed hours since the start go to ``render_frame`` so that a model with drift shows it in the sentinels.

Targets. The target of a capture is ``targets.get(target_id, gap_mm)``. A capture whose gap is None
but whose target definition has a back plate is the open-background variant: the back plate is removed.

Files written next to the frames: manifest.csv, sensor_config.json (the geometry, config id
"synthetic"), registration.json, targets.json, parameters.json, targets_asbuilt.csv (the nominal
values) and session_log.md (what was simulated).
"""
from __future__ import annotations

import math
from dataclasses import asdict
from datetime import datetime, timedelta
from pathlib import Path
from typing import Callable

import numpy as np

from sensorperf.acquisition.plan import PlannedCapture
from sensorperf.geometry.registration import REGISTRATION_FILE_NAME, Registration
from sensorperf.geometry.targets import ASBUILT_FILE_NAME, TARGETS_FILE_NAME, TargetSet
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.manifest import MANIFEST_FILE_NAME, FrameRecord, write_manifest_csv
from sensorperf.io.session import (
    PARAMETERS_FILE_NAME, SENSOR_CONFIG_FILE_NAME, SERIES_DIRS, SESSION_LOG_FILE_NAME, SensorConfig,
    create_session_folder,
)
from sensorperf.parameters import CharacterizationParameters, SensorGeometry
from sensorperf.simulate.sensor_model import SyntheticSensorModel, render_frame, write_frame

SENSOR_CONFIG_ID = "synthetic"
"""sensor_config_id written to every manifest row and to sensor_config.json."""
SESSION_START = datetime(2026, 10, 5, 9, 0, 0)
"""Start of the simulated clock (timestamps of the manifest)."""
SECONDS_PER_HOUR = 3600.0
"""Seconds in an hour (the drift term takes hours)."""
FRAME_HEADER_POSE_KEY = "targetPoseCamera"
"""Header key under which the realized target pose (row-major 4 x 4, mm) is written into each file, for
debugging only; the analyses read the manifest, not the header."""


def _frame_count(capture: PlannedCapture, frame_scale: float) -> int:
    """Frames rendered for a pose: ceil(frames x frame_scale), at least one."""
    return max(1, math.ceil(capture.frames * frame_scale))


def _seed_entropy(rng: np.random.Generator) -> str:
    """The entropy of the generator's seed sequence for the session log (the generator may already
    have been advanced by the plan; the log says so), or "unknown"."""
    seed_sequence = getattr(rng.bit_generator, "seed_seq", None)
    entropy = getattr(seed_sequence, "entropy", None)
    return "unknown" if entropy is None else str(entropy)


def _session_log(root: Path, params: CharacterizationParameters, geometry: SensorGeometry,
                 model: SyntheticSensorModel, registration: Registration, plan: list[PlannedCapture],
                 frame_scale: float, robot_repeatability_mm: float, rng: np.random.Generator,
                 frame_total: int) -> str:
    """Markdown text of session_log.md: what was simulated, with the model values."""
    per_series: dict[str, tuple[int, int]] = {}
    for capture in plan:
        poses, frames = per_series.get(capture.procedure, (0, 0))
        per_series[capture.procedure] = (poses + 1, frames + _frame_count(capture, frame_scale))
    lines = [
        "# Synthetic session",
        "",
        "SIMULATED data from `sensorperf.simulate`; every value below is INDICATIVE and none is a measurement of "
        "the VSX3000. The renderer imitates gross features of the sensor only (see "
        "`sensorperf/simulate/sensor_model.py`).",
        "",
        "## Plan",
        "",
        f"- poses: {len(plan)}, frames written: {frame_total}, frame scale: {frame_scale}",
        f"- image size: {geometry.image_width_px} x {geometry.image_height_px} px, "
        f"f_x = {geometry.sensor_fx_px:.4f} px, baseline = {geometry.sensor_baseline_mm} mm",
        f"- random generator entropy (seed of the generator at creation): {_seed_entropy(rng)}",
        f"- robot repeatability: {robot_repeatability_mm} mm (Gaussian, per axis, on the read-back translation)",
        f"- clock: starts {SESSION_START.isoformat()}, frame rate {geometry.frame_rate_hz} Hz, "
        f"move-and-settle {params.move_and_settle_time_s} s per pose",
        "",
        "| procedure | folder | poses | frames |",
        "| --- | --- | --- | --- |",
    ]
    for procedure, (poses, frames) in per_series.items():
        lines.append(f"| {procedure} | {SERIES_DIRS[procedure]} | {poses} | {frames} |")
    lines += ["", "## Sensor model (indicative)", ""]
    for name, value in asdict(model).items():
        if name != "geometry":
            lines.append(f"- {name}: {value}")
    lines += ["", "## Registration", "",
              f"- camera_to_base: {registration.camera_to_base.as_matrix().reshape(-1).tolist()}",
              f"- target_to_flange: {registration.target_to_flange.as_matrix().reshape(-1).tolist()}",
              "- the ground-truth target pose of every pose is registration.target_to_camera(read-back flange "
              "pose); it is what was rendered and what the manifest holds", ""]
    return "\n".join(lines)


def write_synthetic_session(root: str | Path, params: CharacterizationParameters, geometry: SensorGeometry,
                            model: SyntheticSensorModel, registration: Registration, targets: TargetSet,
                            plan: list[PlannedCapture], rng: np.random.Generator, frame_scale: float = 1.0,
                            progress: Callable[[int, int], None] | None = None,
                            robot_repeatability_mm: float = 0.0) -> Path:
    """Render ``plan`` into a session folder at ``root`` (module docstring) and return the folder.

    ``frame_scale`` < 1 reduces the frames per pose (ceil, at least one) for quick runs;
    ``robot_repeatability_mm`` is the standard deviation of the Gaussian noise added to each axis of the
    read-back flange translation (0 = perfect robot); ``progress(index, total)`` is called after each pose
    (index counts from 1). ``model.geometry`` should be ``geometry``; the frames are rendered with the model."""
    if frame_scale <= 0.0:
        raise ValueError("frame_scale must be positive")
    root = create_session_folder(root)
    ordered = sorted(plan, key=lambda capture: capture.order)
    frame_period_s = 1.0 / float(geometry.require("frame_rate_hz"))
    clock = SESSION_START
    records: list[FrameRecord] = []
    for index, capture in enumerate(ordered, start=1):
        target = targets.get(capture.target_id, capture.gap_mm)
        if capture.gap_mm is None:
            target = target.with_gap(None)          # open-background variant, or a target without a back plate
        # Commanded flange pose, read-back with the robot's repeatability, and the pose that is realized.
        commanded = registration.flange_to_base_for(capture.target_to_camera)
        read_back = RigidTransform(
            commanded.rotation,
            commanded.translation + (rng.normal(0.0, robot_repeatability_mm, 3) if robot_repeatability_mm > 0.0
                                     else 0.0))
        realized = registration.target_to_camera(read_back)
        frames = _frame_count(capture, frame_scale)
        for frame_index in range(frames):
            stamp = clock + timedelta(seconds=frame_index * frame_period_s)
            elapsed_hours = (stamp - SESSION_START).total_seconds() / SECONDS_PER_HOUR
            rendered = render_frame(model, target, realized, rng, elapsed_hours=elapsed_hours)
            path = root / SERIES_DIRS[capture.procedure] / capture.file_name(frame_index)
            write_frame(path, rendered, geometry,
                        {FRAME_HEADER_POSE_KEY: realized.as_matrix().reshape(-1).tolist()})
            records.append(FrameRecord(
                path=path, procedure=capture.procedure, target_id=capture.target_id, gap_mm=capture.gap_mm,
                station_z_mm=capture.station_z_mm, field=capture.field, pose_index=capture.pose_index,
                frame_index=frame_index, robot_pose=read_back, target_pose_camera=realized, seed=capture.seed,
                offset_h_mm=capture.offset_h_mm, offset_v_mm=capture.offset_v_mm,
                timestamp=stamp.isoformat(), sensor_config_id=SENSOR_CONFIG_ID, subseries=capture.subseries,
                tilt_axis=capture.tilt_axis, tilt_deg=capture.tilt_deg, step_mm=capture.step_mm,
                visit=capture.visit, level_index=capture.level_index))
        # The pose took its frames plus the robot move and settle time before the next one.
        clock += timedelta(seconds=frames * frame_period_s + params.move_and_settle_time_s)
        if progress is not None:
            progress(index, len(ordered))

    write_manifest_csv(root / MANIFEST_FILE_NAME, records)
    SensorConfig(config_id=SENSOR_CONFIG_ID, geometry=geometry,
                 notes="simulated by sensorperf.simulate; indicative values").save(root / SENSOR_CONFIG_FILE_NAME)
    registration.save(root / REGISTRATION_FILE_NAME)
    targets.save(root / TARGETS_FILE_NAME)
    params.to_json(root / PARAMETERS_FILE_NAME)
    targets.write_asbuilt_template(root / ASBUILT_FILE_NAME)
    (root / SESSION_LOG_FILE_NAME).write_text(
        _session_log(root, params, geometry, model, registration, ordered, frame_scale, robot_repeatability_mm,
                     rng, len(records)), encoding="utf-8")
    return root
