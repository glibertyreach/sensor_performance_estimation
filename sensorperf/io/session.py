"""
The session folder of Section 9 and the small JSON records it holds.

    Characterization_<YYYYMMDD>/
      sensor_config.json      registration.json      targets_asbuilt.csv
      targets.json            parameters.json        manifest.csv
      session_log.md          environment_log.csv
      00_registration/  A_noise/  B_edges/  B_zstep/  C_area/  D_detect/  sentinels/
      analysis/               (written by the analyses)

``targets.json`` and ``parameters.json`` are additions to the document's list:
the planner writes them so that the analyses read the same target definitions
and parameter values the captures were planned with.

:class:`Session` loads everything an analysis needs: parameters, sensor
geometry (from sensor_config.json, which holds the † values once Step 4.5 has
filled them in), the registration, the targets with the as-built values
applied, and the manifest records.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from sensorperf.geometry.registration import Registration, REGISTRATION_FILE_NAME
from sensorperf.geometry.targets import ASBUILT_FILE_NAME, TARGETS_FILE_NAME, TargetSet, load_targets_asbuilt
from sensorperf.io.manifest import MANIFEST_FILE_NAME, FrameRecord, load_manifest
from sensorperf.parameters import (
    CharacterizationParameters, PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES, PROCEDURE_NOISE,
    PROCEDURE_REGISTRATION, PROCEDURE_SENTINEL, PROCEDURE_ZSTEP, SensorGeometry,
)

SENSOR_CONFIG_FILE_NAME = "sensor_config.json"
PARAMETERS_FILE_NAME = "parameters.json"
SESSION_LOG_FILE_NAME = "session_log.md"
ENVIRONMENT_LOG_FILE_NAME = "environment_log.csv"
ANALYSIS_DIR_NAME = "analysis"
FORWARD_MODEL_FILE_NAME = "forward_model_parameters.json"
VECTOR_GEOMETRY_FIELDS = ("projector_offset_mm", "camera_2d_offset_mm")
"""SensorGeometry fields that are (H, V, Z) vectors: stored as JSON arrays, loaded as tuples."""
"""File names of Section 9 (and the analysis output folder)."""

SERIES_DIRS = {
    PROCEDURE_REGISTRATION: "00_registration",
    PROCEDURE_NOISE: "A_noise",
    PROCEDURE_EDGES: "B_edges",
    PROCEDURE_ZSTEP: "B_zstep",
    PROCEDURE_AREA: "C_area",
    PROCEDURE_DETECTION: "D_detect",
    PROCEDURE_SENTINEL: "sentinels",
}
"""Capture sub-folder of each procedure letter (Section 9, folder layout)."""

SESSION_DIR_FORMAT = "Characterization_{date}"
"""Session folder name; date is YYYYMMDD."""


@dataclass
class SensorConfig:
    """sensor_config.json: the sensor geometry (Section 2 † values) and every
    depth-processing setting of Step 4.2, recorded, not interpreted."""

    config_id: str = "default"
    geometry: SensorGeometry = field(default_factory=SensorGeometry)
    serial_number: str = ""
    exposure: Any = None
    gain: Any = None
    emitter_power: Any = None
    trigger_mode: str = ""
    filters: dict[str, Any] = field(default_factory=dict)
    """Temporal filter, spatial filter, hole filling, confidence threshold, ... as the SDK names them."""
    notes: str = ""

    def save(self, path: str | Path) -> Path:
        document = asdict(self)
        Path(path).write_text(json.dumps(document, indent=2), encoding="utf-8")
        return Path(path)

    @classmethod
    def load(cls, path: str | Path) -> "SensorConfig":
        document = json.loads(Path(path).read_text(encoding="utf-8"))
        geometry = document.pop("geometry", {}) or {}
        for name in VECTOR_GEOMETRY_FIELDS:               # JSON arrays come back as lists; the dataclass holds tuples
            if geometry.get(name) is not None:
                geometry[name] = tuple(geometry[name])
        return cls(geometry=SensorGeometry(**geometry), **document)


@dataclass
class Session:
    """Everything an analysis reads from a session folder."""

    root: Path
    params: CharacterizationParameters
    sensor: SensorConfig
    registration: Registration | None
    targets: TargetSet
    records: list[FrameRecord]

    @property
    def geometry(self) -> SensorGeometry:
        return self.sensor.geometry

    def analysis_dir(self) -> Path:
        path = self.root / ANALYSIS_DIR_NAME
        path.mkdir(parents=True, exist_ok=True)
        return path

    def series_dir(self, procedure: str) -> Path:
        return self.root / SERIES_DIRS[procedure]

    @classmethod
    def load(cls, root: str | Path, require_registration: bool = False) -> "Session":
        """Load a session folder. Missing optional files (registration, as-built
        record, parameters) fall back to None or to the defaults; a missing
        manifest, sensor config or targets file is an error."""
        root = Path(root)
        params_path = root / PARAMETERS_FILE_NAME
        params = CharacterizationParameters.from_json(params_path) if params_path.exists() else CharacterizationParameters()
        sensor = SensorConfig.load(root / SENSOR_CONFIG_FILE_NAME)
        registration_path = root / REGISTRATION_FILE_NAME
        registration = Registration.load(registration_path) if registration_path.exists() else None
        if registration is None and require_registration:
            raise FileNotFoundError(f"{registration_path} is missing; run the registration solve first")
        targets = TargetSet.load(root / TARGETS_FILE_NAME)
        asbuilt_path = root / ASBUILT_FILE_NAME
        if asbuilt_path.exists():
            targets.apply_asbuilt(load_targets_asbuilt(asbuilt_path))
        records = load_manifest(root / MANIFEST_FILE_NAME)
        return cls(root=root, params=params, sensor=sensor, registration=registration, targets=targets,
                   records=records)


def create_session_folder(root: str | Path) -> Path:
    """Create the session folder and its series sub-folders (Section 9)."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    for name in SERIES_DIRS.values():
        (root / name).mkdir(exist_ok=True)
    (root / ANALYSIS_DIR_NAME).mkdir(exist_ok=True)
    return root
