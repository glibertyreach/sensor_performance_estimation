# Reused from the depth_calibration_from_spherical_target repository (sphcal/io/matcloud.py), import path
# adjusted to this package. Keep in step with that repository; fix upstream and re-copy.
# Vendored verbatim (import path adjusted) from the 6DOF-via-2D-and-3D repository,
# sixdof/io/, where it was verified byte-for-byte against 30 real VSX3000 captures.
# Do not edit here; fix upstream and re-vendor.
"""
Reader/writer for Liberty Reach's ".mc" ("Matrix Cloud") file format.

See ``docs/matcloud_format.md`` for the full byte-level layout, how it was
verified (empirically, against 20 real VSX3000 BrownBoard repeatability
captures -- Index00, Index07-Index20, Index25-Index29 -- read byte-by-byte
in a scratch script until every byte of five representative files was
accounted for, then cross-checked against the remaining 15), and the
history of what was previously assumed before a real ``.mc`` file was
available (short version: the header turned out to be **JSON text**, not a
``QVariantHash``, and each ``cv::Mat`` carries an explicit redundant byte
count ahead of its raw pixel payload that the earlier guess did not
expect).

Top-level layout (verified empirically; see ``docs/matcloud_format.md``)::

    file bytes = qCompress(QDataStream bytes of: header, then data)

where ``header`` is (quint8 format tag, quint32 schema tag, quint32 JSON
byte length, that many UTF-8 JSON bytes -- decoded into a plain Python
``dict``, see :func:`_read_header`) and ``data`` is an ``MCData``, i.e. a
``QMap<QString, cv::Mat>`` (``quint32`` count of (``QString`` key,
serialized ``cv::Mat``) pairs, see :func:`_read_cv_mat` for the verified
``cv::Mat`` layout).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from sensorperf.io.qt_datastream import QtDataStreamReader, QtDataStreamWriter, q_compress, q_uncompress

# ---------------------------------------------------------------------------
# cv::Mat type codes (OpenCV's own public "depth | ((channels-1) << shift)"
# encoding, documented in opencv2/core/hal/interface.h -- NOT a Liberty
# Reach convention, just OpenCV's).
# ---------------------------------------------------------------------------
CV_CN_SHIFT = 3
CV_DEPTH_MASK = 7

_CV_DEPTH_TO_DTYPE: dict[int, type] = {
    0: np.uint8,  # CV_8U
    1: np.int8,  # CV_8S
    2: np.uint16,  # CV_16U
    3: np.int16,  # CV_16S
    4: np.int32,  # CV_32S
    5: np.float32,  # CV_32F
    6: np.float64,  # CV_64F
    7: np.float16,  # CV_16F
}
_DTYPE_TO_CV_DEPTH: dict[np.dtype, int] = {np.dtype(v): k for k, v in _CV_DEPTH_TO_DTYPE.items()}


def cv_type_to_dtype_and_channels(cv_type: int) -> tuple[np.dtype, int]:
    """Decode an OpenCV ``cv::Mat::type()`` integer (e.g. ``CV_32FC3``) into
    a numpy dtype and a channel count."""
    depth = cv_type & CV_DEPTH_MASK
    channels = 1 + (cv_type >> CV_CN_SHIFT)
    if depth not in _CV_DEPTH_TO_DTYPE:
        raise ValueError(f"unsupported cv::Mat depth code {depth} (from cv type {cv_type})")
    return np.dtype(_CV_DEPTH_TO_DTYPE[depth]), channels


def dtype_and_channels_to_cv_type(dtype: np.dtype, channels: int) -> int:
    """Inverse of :func:`cv_type_to_dtype_and_channels`."""
    dtype = np.dtype(dtype)
    if dtype not in _DTYPE_TO_CV_DEPTH:
        raise ValueError(f"no cv::Mat depth code for numpy dtype {dtype}")
    if channels < 1 or channels > 4:
        raise ValueError(f"cv::Mat supports 1-4 channels, got {channels}")
    return _DTYPE_TO_CV_DEPTH[dtype] | ((channels - 1) << CV_CN_SHIFT)


# ---------------------------------------------------------------------------
# Header: quint8 format tag, quint32 schema tag, quint32 JSON byte length,
# UTF-8 JSON text -- VERIFIED byte-for-byte against 20 real ".mc" files
# (see docs/matcloud_format.md). This REPLACES the earlier, never-verified
# assumption that the header was a QVariantHash (Qt's binary map encoding);
# it is in fact the Dictionary's own JSON serialization (Dictionary.cpp, per
# the C++ sources reviewed for the earlier, pre-real-file version of this
# module), written inline into the same QDataStream MC::toFile also uses for
# the MCData that follows.
# ---------------------------------------------------------------------------
HEADER_JSON_TEXT_ENCODING = "utf-8"
"""The header JSON's byte encoding. Verified: every JSON header inspected
(20/20 real files) decodes cleanly as UTF-8 (indeed, as plain ASCII -- no
non-ASCII bytes were observed in any header), unlike QString elsewhere in
this format, which is UTF-16BE (a Qt convention, not used for this JSON
blob)."""

DEFAULT_HEADER_FORMAT_TAG = 2
"""The quint8 byte immediately preceding the header's quint32 schema tag,
in every one of the 20 real ".mc" files inspected (Index00, Index07-20,
Index25-29 -- all one repeatability session, so this is 20 SAMPLES OF ONE
SESSION, not 20 independent confirmations of the tag being fixed across
sessions/firmware versions). Its exact meaning could not be pinned down
(no C++ source for the writer of this specific prefix was available -- see
docs/matcloud_format.md): it happens to equal both
:data:`DEFAULT_HEADER_SCHEMA_TAG` below and the JSON header's own
``"version"`` field (also 2 in every file seen), so it may simply be a
duplicate/parallel version marker rather than a semantically distinct
field. :func:`read_matcloud` reads and preserves whatever value is
actually present (see :attr:`MatCloud.header_format_tag`); this constant is
only the *default* :func:`write_matcloud` uses when the caller does not
specify one, chosen to match every real file seen so a synthetic file this
module writes looks like a real one."""

DEFAULT_HEADER_SCHEMA_TAG = 2
"""The quint32 value immediately preceding the header JSON's byte length,
in every one of the 20 real ".mc" files inspected. See
:data:`DEFAULT_HEADER_FORMAT_TAG` for the same uncertainty and single-session
caveat; this field is 4 bytes instead of 1."""


def _read_header(reader: QtDataStreamReader) -> tuple[int, int, dict[str, Any]]:
    """Decode the (format tag, schema tag, JSON header dict) triple; see the
    module-level constants above for what is and is not known about the two
    numeric tags."""
    header_format_tag = reader.read_quint8()
    header_schema_tag = reader.read_quint32()
    json_byte_length = reader.read_quint32()
    json_bytes = reader.read_raw_bytes(json_byte_length)
    header = json.loads(json_bytes.decode(HEADER_JSON_TEXT_ENCODING))
    return header_format_tag, header_schema_tag, header


def _write_header(
    writer: QtDataStreamWriter,
    header: dict[str, Any],
    header_format_tag: int,
    header_schema_tag: int,
) -> None:
    """Inverse of :func:`_read_header`. Uses compact JSON (no whitespace
    between tokens), matching the real files inspected, though this is
    cosmetic -- ``json.loads`` does not care either way, and no consumer of
    this module reads the raw JSON bytes directly."""
    writer.write_quint8(header_format_tag)
    writer.write_quint32(header_schema_tag)
    json_bytes = json.dumps(header, separators=(",", ":")).encode(HEADER_JSON_TEXT_ENCODING)
    writer.write_quint32(len(json_bytes))
    writer.write_raw_bytes(json_bytes)


def _read_cv_mat(reader: QtDataStreamReader) -> np.ndarray:
    """
    Decode one ``cv::Mat`` channel of an MCData.

    VERIFIED (see docs/matcloud_format.md) against 20 real ".mc" files: the
    field order is ``cv_type`` FIRST, then ``rows``, then ``cols`` -- NOT
    ``rows, cols, cv_type`` as an earlier, never-verified version of this
    module assumed -- and there is a fourth ``qint32`` field, the raw
    payload's byte count, BEFORE the raw bytes themselves (redundant with
    ``rows * cols * channels * itemsize``, but present on the wire and
    checked here as a consistency guard rather than silently ignored):

        cv_type | rows | cols | byte_count | raw pixel bytes (byte_count)

    The raw pixel bytes are little-endian (confirmed: decoding a real
    file's "XYZ" channel this way produces z values clustered tightly
    around 1000-1100 mm, matching the known "flat board at 1 m" capture
    geometry; decoding big-endian instead produces values of order 1e38 /
    NaN). Channels are interleaved per pixel (row-major, channel-minor),
    i.e. ``array[row, col, channel]`` -- also confirmed against the same
    real "XYZ" data (x, y are small relative to z and vary smoothly across
    the board, as expected for a camera-frame point cloud).
    """
    cv_type = reader.read_qint32()
    rows = reader.read_qint32()
    cols = reader.read_qint32()
    byte_count = reader.read_qint32()
    dtype, channels = cv_type_to_dtype_and_channels(cv_type)
    expected_byte_count = rows * cols * channels * dtype.itemsize
    if byte_count != expected_byte_count:
        raise ValueError(
            f"cv::Mat byte_count field ({byte_count}) does not match "
            f"rows*cols*channels*itemsize ({expected_byte_count}) for "
            f"cv_type={cv_type} rows={rows} cols={cols} channels={channels} "
            f"itemsize={dtype.itemsize} -- possible corruption or an "
            "unexpected cv::Mat layout (row padding?); see docs/matcloud_format.md"
        )
    raw = reader.read_raw_bytes(byte_count)
    array = np.frombuffer(raw, dtype=dtype.newbyteorder("<")).astype(dtype.newbyteorder("="), copy=True)
    if channels == 1:
        return array.reshape(rows, cols)
    return array.reshape(rows, cols, channels)


def _write_cv_mat(writer: QtDataStreamWriter, array: np.ndarray) -> None:
    """Inverse of :func:`_read_cv_mat`; see its docstring for the (now
    verified) layout."""
    array = np.asarray(array)
    if array.ndim == 2:
        rows, cols = array.shape
        channels = 1
    elif array.ndim == 3:
        rows, cols, channels = array.shape
    else:
        raise ValueError(f"cv::Mat-backed arrays must be 2-D or 3-D, got shape {array.shape}")
    cv_type = dtype_and_channels_to_cv_type(array.dtype, channels)
    byte_count = rows * cols * channels * array.dtype.itemsize
    writer.write_qint32(cv_type)
    writer.write_qint32(rows)
    writer.write_qint32(cols)
    writer.write_qint32(byte_count)
    little_endian = np.ascontiguousarray(array).astype(array.dtype.newbyteorder("<"), copy=False)
    writer.write_raw_bytes(little_endian.tobytes())


def _read_mcdata(reader: QtDataStreamReader) -> dict[str, np.ndarray]:
    count = reader.read_quint32()
    matrices: dict[str, np.ndarray] = {}
    for _ in range(count):
        key = reader.read_qstring()
        matrices[key] = _read_cv_mat(reader)
    return matrices


def _write_mcdata(writer: QtDataStreamWriter, matrices: dict[str, np.ndarray]) -> None:
    writer.write_quint32(len(matrices))
    # QMap iterates in sorted-key order; sorting here makes the output
    # deterministic (real-file byte-for-byte fidelity cannot be verified
    # regardless, since the real writer's exact key iteration order was
    # never observed to matter -- only the decoded dict is ever compared).
    for key in sorted(matrices):
        writer.write_qstring(key)
        _write_cv_mat(writer, matrices[key])


GRAY_CHANNEL_PREFERENCE = ("G", "W", "LeftIR", "RightIR")
"""Matrix names tried, in order, by MatCloud.gray_image()."""


@dataclass
class MatCloud:
    """
    One decoded ".mc" file: a header ``dict`` (the Dictionary's JSON,
    decoded -- keys are plain ``str``, values are whatever
    :func:`json.loads` produced: ``int``, ``float``, ``str``, ``list``,
    ``dict``, ``bool`` or ``None``) and a named set of ``cv::Mat`` channels
    (plain Python ``dict`` of numpy arrays), plus convenience accessors.

    ``header_format_tag``/``header_schema_tag`` are the two numeric values
    that precede the header JSON on the wire (see
    :data:`DEFAULT_HEADER_FORMAT_TAG` for what is and is not known about
    them); they are captured here, rather than folded into ``header``,
    because they are not part of the JSON Dictionary itself.
    """

    header: dict[str, Any]
    matrices: dict[str, np.ndarray]
    header_format_tag: int = DEFAULT_HEADER_FORMAT_TAG
    header_schema_tag: int = DEFAULT_HEADER_SCHEMA_TAG

    def depth_image(self) -> np.ndarray:
        """
        (rows, cols) depth in millimetres, float64.

        Prefers the "XYZ" channel's z component (``CV_32FC3``, so the most
        precise copy available) and falls back to the coarser "Z" channel
        (``CV_16SC1``) when "XYZ" is absent -- "Z" has never actually been
        observed in a real file (all 20 real captures inspected carry only
        "XYZ" and "W"), so this fallback remains unverified, kept only for
        forward/backward compatibility with other capture configurations.

        *** UNIT: VERIFIED millimetres ***. The real "XYZ" data's z values
        for a flat board reported (by the person supplying these captures)
        to be at "1 m" cluster at 1082-1091 mm at the five
        testZRepeatabilityBrownBoard.py box centres (see
        tests/test_matcloud.py and docs/matcloud_format.md) -- consistent
        with millimetres and with a true board distance close to, but not
        exactly, 1000 mm.
        """
        if "XYZ" in self.matrices:
            return np.asarray(self.matrices["XYZ"][..., 2], dtype=np.float64)
        if "Z" in self.matrices:
            return np.asarray(self.matrices["Z"], dtype=np.float64)
        raise KeyError("MatCloud has neither an 'XYZ' nor a 'Z' channel")

    def xyz_image(self) -> np.ndarray:
        """(rows, cols, 3) camera-frame points, float32, as stored (see
        :meth:`depth_image` for the unit note on the z component)."""
        if "XYZ" not in self.matrices:
            raise KeyError("MatCloud has no 'XYZ' channel")
        return self.matrices["XYZ"]

    def gray_image(self) -> np.ndarray:
        """
        (rows, cols) intensity/reflectance channel, as stored.

        *** CHANNEL NAME UPDATED ***: all 20 real files inspected carry
        this channel under the key "W" (``CV_8UC1``), not "G"
        (``CV_16UC1``) as an earlier, pre-real-file version of this module
        assumed (from ``MatCloud.py``'s accessor naming). Both are checked
        here, "G" preferred (in case some other capture configuration uses
        it), falling back to "W" (the key actually observed).
        """
        # Preference order: "G" (legacy 16-bit name), "W" (the 640x480 unit's
        # 8-bit intensity), then the stereo infrared images "LeftIR" /
        # "RightIR" (the 1280x960 unit stores those instead of "W").
        for key in GRAY_CHANNEL_PREFERENCE:
            if key in self.matrices:
                return self.matrices[key]
        raise KeyError(f"MatCloud has none of the gray channels {GRAY_CHANNEL_PREFERENCE}")

    def color_image(self) -> np.ndarray:
        """(rows, cols, 4) BGRA color channel, uint8, as stored. Never
        observed in a real file (all 20 inspected carry only "XYZ"/"W"),
        so this accessor is untested against real data; kept for capture
        configurations that do record color."""
        if "RGBA" not in self.matrices:
            raise KeyError("MatCloud has no 'RGBA' channel")
        return self.matrices["RGBA"]

    def intrinsics(self) -> tuple[float, float, float, float]:
        """(fx, fy, cx, cy). VERIFIED present in every real header
        inspected (20/20) -- resolving the earlier open question of
        whether these four keys are ever actually set."""
        return (
            float(self.header["fx"]),
            float(self.header["fy"]),
            float(self.header["cx"]),
            float(self.header["cy"]),
        )

    def registration(self) -> np.ndarray:
        """(4, 4) registration transform, row-major. VERIFIED: the real
        header's "registration" key is already a flat 16-entry list (row-
        major, ``list[i] == T(i // 4, i % 4)``), not a 12-entry list needing
        an implicit identity 4th row appended as an earlier, pre-real-file
        version of this module's documentation speculated; every real file
        inspected carries the 4x4 identity here (a stationary rig with no
        external tracking feeding this field)."""
        flat = self.header["registration"]
        if len(flat) != 16:
            raise ValueError(f"header['registration'] must have 16 entries, got {len(flat)}")
        return np.asarray(flat, dtype=np.float64).reshape(4, 4)

    def invalid_depth_mask(self) -> np.ndarray:
        """
        True where the depth pixel was never validly read by the sensor.

        *** SENTINEL: VERIFIED exactly 0.0 ***. In every real "XYZ" channel
        inspected, pixels with z == 0.0 also have x == 0.0 and y == 0.0
        (the whole point, not just its depth, is zeroed) and no negative or
        NaN z values were ever observed, so "z <= 0" and "z == 0" select the
        identical set of pixels on real data; this mirrors the (<=)
        convention the user's own testZRepeatabilityBrownBoard.py uses
        (``getBoxPatch``'s ``patch[patch <= 0] = np.nan``).
        """
        return self.depth_image() <= 0.0


def read_matcloud(path: str | Path) -> MatCloud:
    """Read and decode a ".mc" file (see the module docstring for the
    verified layout)."""
    raw_file_bytes = Path(path).read_bytes()
    decompressed = q_uncompress(raw_file_bytes)
    if not decompressed:
        raise ValueError(f"{path}: qUncompress produced no data (not a qCompress-compressed .mc file?)")
    reader = QtDataStreamReader(decompressed)
    header_format_tag, header_schema_tag, header = _read_header(reader)
    matrices = _read_mcdata(reader)
    return MatCloud(
        header=header,
        matrices=matrices,
        header_format_tag=header_format_tag,
        header_schema_tag=header_schema_tag,
    )


def write_matcloud(
    path: str | Path,
    header: dict[str, Any],
    matrices: dict[str, np.ndarray],
    header_format_tag: int = DEFAULT_HEADER_FORMAT_TAG,
    header_schema_tag: int = DEFAULT_HEADER_SCHEMA_TAG,
) -> None:
    """
    Encode and write a ".mc" file: the exact inverse of :func:`read_matcloud`
    (``read_matcloud(write_matcloud(path, header, matrices))`` reproduces
    ``header``/``matrices``/the two tags exactly, up to ``json``'s float
    formatting -- Python's ``json`` module round-trips every IEEE-754
    double it can produce, which covers everything :func:`json.loads` could
    have handed back in the first place -- and up to numpy dtype
    round-tripping through whatever ``cv::Mat`` depth code
    :func:`dtype_and_channels_to_cv_type` maps it to). Primarily meant for
    synthetic test fixtures; see ``docs/matcloud_format.md`` for how this
    layout was verified against real files.
    """
    writer = QtDataStreamWriter()
    _write_header(writer, header, header_format_tag, header_schema_tag)
    _write_mcdata(writer, matrices)
    compressed = q_compress(writer.getvalue(), level=9)
    Path(path).write_bytes(compressed)
