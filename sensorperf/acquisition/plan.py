"""
Station and pose planning for Part I of the procedure (Sections 4 to 9).

A plan is a list of :class:`PlannedCapture`: every commanded pose of every
series, with its Section 9 identity (procedure, target, gap, station, field,
pose index), its sub-series label, the frame count, the logged random seed and
lateral offset, the tilt or Z-step details, and the ground-truth target pose in
the camera frame that the robot must realize. The synthetic session writer
renders a plan; the robot program executes it; the manifest builder matches the
captured files back to it.

The planning functions (one per series) and the writers follow the dataclass;
their rules are the ones the procedure states step by step, cited in the
docstrings.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.manifest import format_file_name


@dataclass
class PlannedCapture:
    """One commanded pose of the plan (all frames of the pose share it)."""

    procedure: str
    target_id: str
    gap_mm: float | None
    station_z_mm: float
    field: int
    pose_index: int
    frames: int
    subseries: str
    target_to_camera: RigidTransform
    """The wanted ground-truth target pose (target -> camera), offsets and tilt applied."""
    seed: int | None = None
    """Seed of the random draw that produced this pose's offset or its place in the order."""
    offset_h_mm: float = 0.0
    offset_v_mm: float = 0.0
    tilt_axis: str = ""
    tilt_deg: float = 0.0
    step_mm: float | None = None
    """Commanded Z step (B-Z ladder) or staircase position relative to Z0, mm."""
    visit: str = ""
    """A or B for the B-Z ladder alternation."""
    level_index: int | None = None
    """Diameter-ladder level (D) when the pose targets one level; usually None (all levels per frame)."""
    order: int = 0
    """Acquisition order after randomization (0-based, over the whole plan)."""
    notes: dict[str, Any] = field(default_factory=dict)

    def pose_key(self) -> tuple:
        return (self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field, self.pose_index)

    def file_name(self, frame_index: int) -> str:
        """The Section 9 file name of one frame of this pose."""
        return format_file_name(self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field,
                                self.pose_index, frame_index)
