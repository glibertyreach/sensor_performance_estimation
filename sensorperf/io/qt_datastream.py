# Reused from the depth_calibration_from_spherical_target repository (sphcal/io/qt_datastream.py), import path
# adjusted to this package. Keep in step with that repository; fix upstream and re-copy.
# Vendored verbatim (import path adjusted) from the 6DOF-via-2D-and-3D repository,
# sixdof/io/, where it was verified byte-for-byte against 30 real VSX3000 captures.
# Do not edit here; fix upstream and re-vendor.
"""
A minimal reader/writer for Qt5's ``QDataStream`` binary encoding (default
stream version, which is what ``MC::toFile``/``MC::fromFile`` use -- see
``MC.cpp`` in ``docs/matcloud_format.md``), plus Qt's ``qCompress``/
``qUncompress`` container format.

Only the primitives and container types the ".mc" file format actually needs
are implemented (see ``docs/matcloud_format.md`` for exactly which header
keys and value types were observed in the C++ sources); anything else raises
``NotImplementedError`` naming the unsupported type id, rather than silently
misinterpreting the byte stream.

NOTE (post real-file verification -- see ``docs/matcloud_format.md``): the
".mc" file's own header turned out to be plain JSON text, not a
``QVariantHash`` run through this module's ``QVariant``/container
machinery as originally assumed before a real file was available. The
``QVariant``/``QVariantList``/``QVariantMap``/``QVariantHash`` machinery
below is kept because it faithfully documents Qt's own public wire format
(useful in its own right, and exercised by this module's tests against
hand-derived Qt byte sequences) and because ``sixdof.io.matcloud`` still
uses this module's plain ``QString``/primitive readers and writers (e.g. for
each ``cv::Mat`` channel's name) -- just not ``read_qvariant_map`` for the
header any more.

Byte-level conventions (all taken from Qt5's public documentation for
``QDataStream``, not from any Liberty Reach source -- these are Qt's own
wire format and are the same for every Qt application):

* ``QDataStream`` is big-endian by default and this module only implements
  that default (the LRI sources never call ``setByteOrder``).
* Integers (``qint8``/``quint8`` .. ``qint64``/``quint64``) are written as
  their plain big-endian two's-complement/unsigned representation.
* ``bool`` is written as a single byte, 0 or 1.
* Floating point: ``QDataStream`` has a per-stream
  ``FloatingPointPrecision`` setting (``DoublePrecision`` by default since
  Qt 4.6) that controls how a C++ ``float`` or ``double`` value passed to
  ``operator<<``/``operator>>`` is put on the wire -- at the default
  setting BOTH occupy 8 bytes (a ``float`` is promoted to ``double`` before
  being written). None of the C++ sources we could read
  (``MC.cpp``/``MCHelper.cpp``/``ImageHeader.cpp``) call
  ``setFloatingPointPrecision``, so this module assumes the default
  (``read_double``/``write_double``, 8 bytes) everywhere a bare
  ``float``/``double`` is read or written. This is a real, UNVERIFIED
  assumption -- see ``docs/matcloud_format.md``.
* ``QString``: ``quint32`` byte length (not character count) followed by
  the string's UTF-16BE code units; the length ``0xFFFFFFFF`` denotes a
  null (not merely empty) ``QString``, with no following bytes.
* ``QByteArray``: ``quint32`` length followed by the raw bytes; again
  ``0xFFFFFFFF`` denotes null.
* ``QVariant``: ``quint32`` type id (``QMetaType::Type``), ``quint8``
  is-null flag, then the payload in the format for that type id (nested
  ``QVariant``s inside a ``QVariantList``/``QVariantMap`` each repeat this
  three-part encoding). The is-null flag does NOT gate whether a payload
  follows -- only ``QMetaType::UnknownType`` (0, "Invalid") has no payload;
  every other type, null or not, still writes its type's null-safe
  encoding (e.g. a null ``QString`` still writes its own
  ``0xFFFFFFFF`` sentinel).
* ``QVariantList``: ``quint32`` count followed by that many ``QVariant``s.
* ``QVariantMap``/``QVariantHash``: ``quint32`` count followed by that many
  (``QString`` key, ``QVariant`` value) pairs. The two container types
  share this exact wire layout; they differ only in iteration order
  (``QVariantMap`` sorted by key, ``QVariantHash`` unspecified), which is
  irrelevant once decoded into a Python ``dict``.
* ``QStringList``: ``quint32`` count followed by that many ``QString``s.
* A registered C++/``Q_DECLARE_METATYPE`` "user type" (type id 127,
  ``QMetaType::User``) additionally writes a ``QByteArray`` holding the
  type's registered name before its payload, which is then written by
  that type's own registered ``QMetaType`` save function -- a payload
  format this module cannot know in general, so decoding a user type
  raises ``NotImplementedError`` naming it. None of the Dictionary/MCData
  header fields visible in the C++ sources we read use one.
* ``QMetaType::Float`` (type id 135) is a distinct QVariant type from
  ``double`` (used when a bare C++ ``float`` -- e.g. ``MC::setFrameNumber``'s
  argument -- is assigned into a ``QVariant``); unlike the stream's
  ``FloatingPointPrecision`` setting above, ``QMetaType::Float``'s own save
  function always writes exactly 4 bytes. This module implements it
  (``read_float32``/``write_float32``) but it is UNVERIFIED against a real
  file (see ``docs/matcloud_format.md``).

``qCompress``/``qUncompress`` (also Qt public API, documented behaviour):
the compressed buffer is a 4-byte big-endian uncompressed length followed by
a standard zlib stream.
"""

from __future__ import annotations

import struct
import zlib
from typing import Any

# ---------------------------------------------------------------------------
# QVariant type ids (a subset of Qt's QMetaType::Type enum) actually needed
# to round-trip an MC Dictionary header. These numeric values are Qt's own
# published API (QMetaType), not anything specific to the Liberty Reach
# sources; they are stable across Qt4/Qt5.
# ---------------------------------------------------------------------------
QVARIANT_TYPE_INVALID = 0
QVARIANT_TYPE_BOOL = 1
QVARIANT_TYPE_INT = 2
QVARIANT_TYPE_UINT = 3
QVARIANT_TYPE_LONGLONG = 4
QVARIANT_TYPE_ULONGLONG = 5
QVARIANT_TYPE_DOUBLE = 6
QVARIANT_TYPE_QVARIANTMAP = 8
QVARIANT_TYPE_QVARIANTLIST = 9
QVARIANT_TYPE_QSTRING = 10
QVARIANT_TYPE_QSTRINGLIST = 11
QVARIANT_TYPE_QBYTEARRAY = 12
QVARIANT_TYPE_QVARIANTHASH = 28
QVARIANT_TYPE_FLOAT = 135
"""QMetaType::Float; see the module docstring's note on this being distinct
from QDataStream's stream-wide FloatingPointPrecision setting."""
QVARIANT_TYPE_USER = 127

_NULL_LENGTH_MARKER = 0xFFFFFFFF
"""Qt's sentinel quint32 length marking a null (not merely empty) QString or
QByteArray."""


class QtDataStreamReader:
    """Sequentially decodes one in-memory buffer as a Qt5 ``QDataStream``
    (default version, big-endian, default ``DoublePrecision`` floats --
    see the module docstring)."""

    def __init__(self, data: bytes):
        self._data = data
        self._offset = 0

    def remaining(self) -> int:
        """Bytes left unread in the buffer."""
        return len(self._data) - self._offset

    def read_raw_bytes(self, count: int) -> bytes:
        """The next ``count`` raw bytes, unconverted (Qt's ``readRawData``)."""
        if count < 0:
            raise ValueError(f"count must be non-negative, got {count}")
        if self._offset + count > len(self._data):
            raise EOFError(
                f"QtDataStreamReader: need {count} bytes at offset {self._offset}, "
                f"only {self.remaining()} left"
            )
        chunk = self._data[self._offset : self._offset + count]
        self._offset += count
        return chunk

    # ---- fixed-size primitives (all big-endian) ---------------------------
    def read_qint8(self) -> int:
        return struct.unpack(">b", self.read_raw_bytes(1))[0]

    def read_quint8(self) -> int:
        return struct.unpack(">B", self.read_raw_bytes(1))[0]

    def read_qint16(self) -> int:
        return struct.unpack(">h", self.read_raw_bytes(2))[0]

    def read_quint16(self) -> int:
        return struct.unpack(">H", self.read_raw_bytes(2))[0]

    def read_qint32(self) -> int:
        return struct.unpack(">i", self.read_raw_bytes(4))[0]

    def read_quint32(self) -> int:
        return struct.unpack(">I", self.read_raw_bytes(4))[0]

    def read_qint64(self) -> int:
        return struct.unpack(">q", self.read_raw_bytes(8))[0]

    def read_quint64(self) -> int:
        return struct.unpack(">Q", self.read_raw_bytes(8))[0]

    def read_bool(self) -> bool:
        return self.read_quint8() != 0

    def read_double(self) -> float:
        """A bare C++ ``double`` (also what a bare C++ ``float`` reads as
        under the default ``DoublePrecision`` stream setting; see module
        docstring)."""
        return struct.unpack(">d", self.read_raw_bytes(8))[0]

    def read_float32(self) -> float:
        """``QMetaType::Float``'s own fixed 4-byte payload (see module
        docstring); NOT what a bare C++ ``float`` occupies on the wire under
        the default stream precision -- use :meth:`read_double` for that."""
        return struct.unpack(">f", self.read_raw_bytes(4))[0]

    # ---- variable-length containers ---------------------------------------
    def read_qstring(self) -> str | None:
        length = self.read_quint32()
        if length == _NULL_LENGTH_MARKER:
            return None
        return self.read_raw_bytes(length).decode("utf-16-be")

    def read_qbytearray(self) -> bytes | None:
        length = self.read_quint32()
        if length == _NULL_LENGTH_MARKER:
            return None
        return self.read_raw_bytes(length)

    def read_qvariant_list(self) -> list:
        count = self.read_quint32()
        return [self.read_qvariant() for _ in range(count)]

    def read_qvariant_map(self) -> dict:
        count = self.read_quint32()
        result: dict = {}
        for _ in range(count):
            key = self.read_qstring()
            result[key] = self.read_qvariant()
        return result

    # QVariantHash shares QVariantMap's wire layout exactly (see docstring).
    read_qvariant_hash = read_qvariant_map

    def read_qstringlist(self) -> list:
        count = self.read_quint32()
        return [self.read_qstring() for _ in range(count)]

    def read_qvariant(self) -> Any:
        type_id = self.read_quint32()
        is_null = self.read_bool()
        if type_id == QVARIANT_TYPE_INVALID:
            return None
        if type_id == QVARIANT_TYPE_BOOL:
            return self.read_bool()
        if type_id == QVARIANT_TYPE_INT:
            return self.read_qint32()
        if type_id == QVARIANT_TYPE_UINT:
            return self.read_quint32()
        if type_id == QVARIANT_TYPE_LONGLONG:
            return self.read_qint64()
        if type_id == QVARIANT_TYPE_ULONGLONG:
            return self.read_quint64()
        if type_id == QVARIANT_TYPE_DOUBLE:
            return self.read_double()
        if type_id == QVARIANT_TYPE_FLOAT:
            return self.read_float32()
        if type_id == QVARIANT_TYPE_QSTRING:
            return self.read_qstring()
        if type_id == QVARIANT_TYPE_QBYTEARRAY:
            return self.read_qbytearray()
        if type_id == QVARIANT_TYPE_QVARIANTLIST:
            return self.read_qvariant_list()
        if type_id in (QVARIANT_TYPE_QVARIANTMAP, QVARIANT_TYPE_QVARIANTHASH):
            return self.read_qvariant_map()
        if type_id == QVARIANT_TYPE_QSTRINGLIST:
            return self.read_qstringlist()
        if type_id == QVARIANT_TYPE_USER:
            type_name = self.read_qbytearray()
            raise NotImplementedError(
                f"QVariant holds a registered C++ user type {type_name!r}; decoding an "
                "arbitrary user type's QMetaType payload is not implemented (no Dictionary "
                "header field found in the C++ sources uses one -- see docs/matcloud_format.md)"
            )
        _ = is_null  # the null flag never gates whether a payload follows; see docstring
        raise NotImplementedError(
            f"QVariant type id {type_id} is not one this reader supports; see docs/matcloud_format.md"
        )


class QtDataStreamWriter:
    """Builds an in-memory buffer in the same wire format
    :class:`QtDataStreamReader` decodes (see its docstring)."""

    def __init__(self):
        self._chunks: list[bytes] = []

    def getvalue(self) -> bytes:
        return b"".join(self._chunks)

    def write_raw_bytes(self, data: bytes) -> None:
        self._chunks.append(bytes(data))

    # ---- fixed-size primitives ----------------------------------------------
    def write_qint8(self, value: int) -> None:
        self._chunks.append(struct.pack(">b", value))

    def write_quint8(self, value: int) -> None:
        self._chunks.append(struct.pack(">B", value))

    def write_qint16(self, value: int) -> None:
        self._chunks.append(struct.pack(">h", value))

    def write_quint16(self, value: int) -> None:
        self._chunks.append(struct.pack(">H", value))

    def write_qint32(self, value: int) -> None:
        self._chunks.append(struct.pack(">i", value))

    def write_quint32(self, value: int) -> None:
        self._chunks.append(struct.pack(">I", value))

    def write_qint64(self, value: int) -> None:
        self._chunks.append(struct.pack(">q", value))

    def write_quint64(self, value: int) -> None:
        self._chunks.append(struct.pack(">Q", value))

    def write_bool(self, value: bool) -> None:
        self.write_quint8(1 if value else 0)

    def write_double(self, value: float) -> None:
        self._chunks.append(struct.pack(">d", value))

    def write_float32(self, value: float) -> None:
        self._chunks.append(struct.pack(">f", value))

    # ---- variable-length containers ------------------------------------------
    def write_qstring(self, value: str | None) -> None:
        if value is None:
            self.write_quint32(_NULL_LENGTH_MARKER)
            return
        encoded = value.encode("utf-16-be")
        self.write_quint32(len(encoded))
        self.write_raw_bytes(encoded)

    def write_qbytearray(self, value: bytes | None) -> None:
        if value is None:
            self.write_quint32(_NULL_LENGTH_MARKER)
            return
        self.write_quint32(len(value))
        self.write_raw_bytes(bytes(value))

    def write_qvariant_list(self, items: list) -> None:
        self.write_quint32(len(items))
        for item in items:
            self.write_qvariant(item)

    def write_qvariant_map(self, mapping: dict) -> None:
        self.write_quint32(len(mapping))
        for key, value in mapping.items():
            self.write_qstring(key)
            self.write_qvariant(value)

    write_qvariant_hash = write_qvariant_map

    def write_qstringlist(self, items: list) -> None:
        self.write_quint32(len(items))
        for item in items:
            self.write_qstring(item)

    def write_qvariant(self, value: Any) -> None:
        """
        Encode a plain Python value as a QVariant.

        Because Python's type system is coarser than QVariant's, this
        applies a fixed, documented mapping rather than trying to recover
        the exact original C++ type: ``bool`` -> Bool, ``int`` -> Int if it
        fits in 32 bits signed else LongLong, ``float`` -> Double, ``str``
        -> QString, ``bytes``/``bytearray`` -> QByteArray, ``list``/``tuple``
        -> QVariantList (recursively), ``dict`` -> QVariantMap
        (recursively). This is sufficient to round-trip everything
        :func:`sixdof.io.matcloud.write_matcloud` needs to write, but it is
        NOT guaranteed to reproduce the exact QVariant type id a real MC
        writer would have chosen for a given header field.
        """
        if value is None:
            self.write_quint32(QVARIANT_TYPE_INVALID)
            self.write_bool(True)
            return
        if isinstance(value, bool):
            self.write_quint32(QVARIANT_TYPE_BOOL)
            self.write_bool(False)
            self.write_bool(value)
        elif isinstance(value, int):
            if -(2**31) <= value < 2**31:
                self.write_quint32(QVARIANT_TYPE_INT)
                self.write_bool(False)
                self.write_qint32(value)
            else:
                self.write_quint32(QVARIANT_TYPE_LONGLONG)
                self.write_bool(False)
                self.write_qint64(value)
        elif isinstance(value, float):
            self.write_quint32(QVARIANT_TYPE_DOUBLE)
            self.write_bool(False)
            self.write_double(value)
        elif isinstance(value, str):
            self.write_quint32(QVARIANT_TYPE_QSTRING)
            self.write_bool(False)
            self.write_qstring(value)
        elif isinstance(value, (bytes, bytearray)):
            self.write_quint32(QVARIANT_TYPE_QBYTEARRAY)
            self.write_bool(False)
            self.write_qbytearray(bytes(value))
        elif isinstance(value, (list, tuple)):
            self.write_quint32(QVARIANT_TYPE_QVARIANTLIST)
            self.write_bool(False)
            self.write_qvariant_list(list(value))
        elif isinstance(value, dict):
            self.write_quint32(QVARIANT_TYPE_QVARIANTMAP)
            self.write_bool(False)
            self.write_qvariant_map(value)
        else:
            raise TypeError(f"no QVariant encoding rule for Python type {type(value)!r}")


def q_uncompress(data: bytes) -> bytes:
    """
    Qt's ``qUncompress``: the first 4 bytes of ``data`` are the uncompressed
    length, big-endian; the remainder is a standard zlib stream (Qt calls
    plain zlib ``inflate`` under the hood). Returns ``b""`` for an empty or
    zero-length-header input, matching Qt's own "returns an empty QByteArray
    on error" behaviour for the degenerate case.
    """
    if len(data) < 4:
        return b""
    uncompressed_length = struct.unpack(">I", data[:4])[0]
    if uncompressed_length == 0:
        return b""
    payload = zlib.decompress(bytes(data[4:]))
    if len(payload) != uncompressed_length:
        raise ValueError(
            f"qUncompress: header declared {uncompressed_length} bytes but zlib produced {len(payload)}"
        )
    return payload


def q_compress(data: bytes, level: int = -1) -> bytes:
    """
    Qt's ``qCompress``: a 4-byte big-endian uncompressed length followed by
    a standard zlib stream. ``level`` follows zlib's ``-1..9`` convention
    (-1 = zlib default); ``MC::toFile`` always calls ``qCompress(data, 9)``.
    """
    if not (-1 <= level <= 9):
        raise ValueError(f"level must be between -1 and 9 (zlib convention), got {level}")
    data = bytes(data)
    compressed = zlib.compress(data, level)
    return struct.pack(">I", len(data)) + compressed
