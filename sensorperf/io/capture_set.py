"""
Frames of one commanded pose loaded together as a stack (adapted from the
calibration repository's capture_set.py for the characterization manifest).

A PoseStack holds the depth images of all frames of one pose (camera z in mm,
NaN where the sensor delivered no read), the camera-frame XYZ images, a
validity mask, the first frame's header and the pinhole camera built from it,
and the manifest records of the frames in frame order.

The zero sentinel of the ``.mc`` format (the whole XYZ point is zero where the
pixel was not read; verified against real VSX3000 captures in the vendored
reader) is converted to NaN here, once, so that every analysis can use NaN
arithmetic (Section 10, Step 1).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np

from sensorperf.geometry.camera import PinholeCamera
from sensorperf.io.manifest import FrameRecord, group_by_pose
from sensorperf.io.matcloud import read_matcloud

XYZ_CHANNEL_NAME = "XYZ"
"""Name of the matrix holding the (H, W, 3) camera-frame points in a ``.mc`` file."""


@dataclass
class PoseStack:
    """All frames of one pose.

    depth:   (F, H, W) float64 camera-z in mm, NaN where not read
    xyz:     (F, H, W, 3) float32 camera-frame points in mm (zeros where not read, as stored)
    valid:   (F, H, W) bool, True where z > 0
    header:  header dict of the first frame
    camera:  PinholeCamera from that header and the array shape
    records: the F manifest records, in frame order
    """

    depth: np.ndarray
    xyz: np.ndarray
    valid: np.ndarray
    header: dict
    camera: PinholeCamera
    records: list[FrameRecord] = field(default_factory=list)

    @property
    def frame_count(self) -> int:
        return int(self.depth.shape[0])

    def record(self) -> FrameRecord:
        """The first frame's record (pose-level fields are shared by all frames)."""
        return self.records[0]

    def mean_depth(self) -> np.ndarray:
        """Per-pixel mean over the frames where the pixel was read (NaN where never read)."""
        # A pixel never read in any frame has an all-NaN column; numpy warns about the empty mean,
        # which is the intended NaN result, so the warning is silenced here.
        import warnings
        with np.errstate(invalid="ignore"), warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            return np.nanmean(self.depth, axis=0)


def load_frame_depth(path: str | Path) -> tuple[np.ndarray, np.ndarray, dict]:
    """(depth with NaN for no-reads, xyz as stored, header) of one capture file."""
    capture = read_matcloud(path)
    xyz = np.asarray(capture.matrices[XYZ_CHANNEL_NAME], dtype=np.float32)
    z = np.asarray(xyz[..., 2], dtype=np.float64)
    depth = np.where(z > 0.0, z, np.nan)
    return depth, xyz, capture.header


def load_stack(records: Iterable[FrameRecord]) -> PoseStack:
    """Read the frames of one pose (records sorted by frame index). Raises
    ValueError if the records belong to different poses or differ in image size."""
    records = sorted(records, key=lambda r: r.frame_index)
    if not records:
        raise ValueError("load_stack needs at least one record")
    keys = {r.pose_key() for r in records}
    if len(keys) != 1:
        raise ValueError(f"records of {len(keys)} different poses were passed to load_stack")
    depths, xyzs = [], []
    first_header: dict | None = None
    for record in records:
        depth, xyz, header = load_frame_depth(record.path)
        if depths and depth.shape != depths[0].shape:
            raise ValueError(f"pose {record.pose_key()}: mixed image sizes {depths[0].shape} vs {depth.shape}")
        if first_header is None:
            first_header = header
        depths.append(depth)
        xyzs.append(xyz)
    depth_stack = np.stack(depths, axis=0)
    xyz_stack = np.stack(xyzs, axis=0)
    height, width = depth_stack.shape[1], depth_stack.shape[2]
    camera = PinholeCamera.from_matcloud_header(first_header, width_px=width, height_px=height)
    return PoseStack(depth=depth_stack, xyz=xyz_stack, valid=np.isfinite(depth_stack), header=first_header,
                     camera=camera, records=records)


def iter_pose_stacks(records: Iterable[FrameRecord]):
    """Yield (pose_key, PoseStack) for every pose among the records, in order of first appearance."""
    for key, group in group_by_pose(records).items():
        yield key, load_stack(group)


def camera_from_record(record: FrameRecord) -> PinholeCamera:
    """The pinhole camera of a capture file (header intrinsics and array size), without the depth."""
    depth, _, header = load_frame_depth(record.path)
    return PinholeCamera.from_matcloud_header(header, width_px=depth.shape[1], height_px=depth.shape[0])
