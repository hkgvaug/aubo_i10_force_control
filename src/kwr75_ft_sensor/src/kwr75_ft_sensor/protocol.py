#!/usr/bin/env python3
"""KWR75-RS422 frame decoding.

Pure Python: no ROS or pyserial dependency, so the parser can be unit tested
on any machine.

Frame layout (28 bytes, from KWR75 manual pages 8-9):

    offset  size  content
    ------  ----  ---------------------------------------------
    0       1     header: 0x48 (stream) or 0x49 (single shot)
    1       1     0xAA
    2       4     Fx, IEEE-754 float32, little endian, unit kgf
    6       4     Fy, kgf
    10      4     Fz, kgf
    14      4     Mx, kgf*m
    18      4     My, kgf*m
    22      4     Mz, kgf*m
    26      2     0x0D 0x0A

The manual's example frame is used in the unit tests. Note that the manual
prints Fx as +9.7787 kgf although the float bytes decode to -9.7787 kgf.
"""

import math
import struct
from collections import namedtuple

#: 1 kgf = 9.80665 N. The manual reports both force (Kg) and torque (Kgm)
#: in kilogram-force units.
GRAVITY = 9.80665

FRAME_SIZE = 28
HEADER_BYTES = (0x48, 0x49)
SYNC_BYTE = 0xAA
TERMINATOR = b"\r\n"
FLOAT_FORMAT = "<6f"

Kwr75Sample = namedtuple("Kwr75Sample", "header raw fx fy fz mx my mz")


class Kwr75FrameParser(object):
    """Incrementally reassemble KWR75 frames from arbitrary serial chunks.

    ``feed()`` returns a list of :class:`Kwr75Sample` objects. Values are
    already converted to SI units: force in N, torque in N*m.
    """

    def __init__(self, max_buffer=4096):
        self._buffer = bytearray()
        self.max_buffer = max_buffer
        self.frames_ok = 0
        self.bad_terminator = 0
        self.bad_payload = 0
        self.discarded_bytes = 0

    def feed(self, data):
        if data:
            self._buffer.extend(data)
        return self._drain()

    def reset(self):
        self._buffer.clear()

    def _drain(self):
        samples = []
        while True:
            start = self._find_header()
            if start is None:
                self._discard_without_header()
                return samples

            if start:
                self.discarded_bytes += start
                del self._buffer[:start]

            if len(self._buffer) < FRAME_SIZE:
                return samples

            frame = bytes(self._buffer[:FRAME_SIZE])

            if frame[26:28] != TERMINATOR:
                # Header matched but the frame boundary is wrong: drop one
                # byte and look for the next header.
                self.bad_terminator += 1
                del self._buffer[0]
                self.discarded_bytes += 1
                continue

            values = struct.unpack(FLOAT_FORMAT, frame[2:26])
            if not all(math.isfinite(value) for value in values):
                self.bad_payload += 1
                del self._buffer[:FRAME_SIZE]
                continue

            del self._buffer[:FRAME_SIZE]
            self.frames_ok += 1
            samples.append(Kwr75Sample(
                frame[0],
                frame,
                values[0] * GRAVITY,
                values[1] * GRAVITY,
                values[2] * GRAVITY,
                values[3] * GRAVITY,
                values[4] * GRAVITY,
                values[5] * GRAVITY,
            ))

            if len(self._buffer) > self.max_buffer:
                self.discarded_bytes += len(self._buffer)
                self._buffer.clear()
                return samples

    def _find_header(self):
        buffer = self._buffer
        for index in range(len(buffer) - 1):
            if buffer[index] in HEADER_BYTES and buffer[index + 1] == SYNC_BYTE:
                return index
        return None

    def _discard_without_header(self):
        # A header may be split across two reads, so keep a trailing byte
        # that could still become 0x48/0x49 followed by 0xAA.
        buffer = self._buffer
        if not buffer:
            return
        keep = 1 if buffer[-1] in HEADER_BYTES else 0
        drop = len(buffer) - keep
        if drop:
            self.discarded_bytes += drop
            del buffer[:drop]
