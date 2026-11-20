#!/usr/bin/env python3
"""Unit tests for the pure-Python KWR75 frame parser."""

import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from kwr75_ft_sensor.protocol import GRAVITY, Kwr75FrameParser  # noqa: E402

# Example frame from the manual, page 8.
FRAME_HEX = "48AAB0751CC19B3FEDC0AE5CF040D4B52DBE5F55993E7C1AB0BB0D0A"
FRAME = bytes.fromhex(FRAME_HEX)

EXPECTED_KGF = (-9.778732299804688, -7.4140143394470215, 7.511313438415527,
                -0.16963893175125122, 0.2994794547557831,
                -0.0053742509335279465)


class FrameParserTest(unittest.TestCase):
    def assertFrameValues(self, sample):
        actual = (sample.fx, sample.fy, sample.fz,
                  sample.mx, sample.my, sample.mz)
        expected = [value * GRAVITY for value in EXPECTED_KGF]
        for got, want in zip(actual, expected):
            self.assertAlmostEqual(got, want, places=4)

    def test_manual_example_frame(self):
        parser = Kwr75FrameParser()
        samples = parser.feed(FRAME)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].header, 0x48)
        self.assertEqual(samples[0].raw, FRAME)
        self.assertFrameValues(samples[0])
        self.assertEqual(parser.frames_ok, 1)

    def test_split_across_reads(self):
        parser = Kwr75FrameParser()
        self.assertEqual(parser.feed(FRAME[:7]), [])
        self.assertEqual(parser.feed(FRAME[7:20]), [])
        samples = parser.feed(FRAME[20:])
        self.assertEqual(len(samples), 1)
        self.assertFrameValues(samples[0])

    def test_garbage_prefix(self):
        parser = Kwr75FrameParser()
        samples = parser.feed(b"\x00\x11\x22\x33" + FRAME)
        self.assertEqual(len(samples), 1)
        self.assertEqual(parser.discarded_bytes, 4)
        self.assertFrameValues(samples[0])

    def test_two_frames_in_one_chunk(self):
        parser = Kwr75FrameParser()
        samples = parser.feed(FRAME + FRAME)
        self.assertEqual(len(samples), 2)
        for sample in samples:
            self.assertFrameValues(sample)

    def test_bad_terminator_resynchronises(self):
        bad = bytearray(FRAME)
        bad[26:28] = b"\x00\x00"
        parser = Kwr75FrameParser()
        samples = parser.feed(bytes(bad) + FRAME)
        self.assertEqual(len(samples), 1)
        self.assertFrameValues(samples[0])
        self.assertGreaterEqual(parser.bad_terminator, 1)

    def test_nan_payload_is_dropped(self):
        bad = bytearray(FRAME)
        bad[2:6] = struct.pack("<f", float("nan"))
        parser = Kwr75FrameParser()
        self.assertEqual(parser.feed(bytes(bad)), [])
        self.assertEqual(parser.bad_payload, 1)
        samples = parser.feed(FRAME)
        self.assertEqual(len(samples), 1)
        self.assertFrameValues(samples[0])

    def test_single_shot_header_0x49(self):
        frame = bytearray(FRAME)
        frame[0] = 0x49
        parser = Kwr75FrameParser()
        samples = parser.feed(bytes(frame))
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].header, 0x49)
        self.assertTrue(all(math.isfinite(v) for v in
                            (samples[0].fx, samples[0].fy, samples[0].fz,
                             samples[0].mx, samples[0].my, samples[0].mz)))

    def test_partial_header_at_buffer_end(self):
        parser = Kwr75FrameParser()
        self.assertEqual(parser.feed(b"\x00\x48"), [])
        samples = parser.feed(b"\xaa" + FRAME[2:])
        self.assertEqual(len(samples), 1)
        self.assertFrameValues(samples[0])

    def test_noise_only_is_discarded(self):
        parser = Kwr75FrameParser()
        self.assertEqual(parser.feed(b"\x01\x02\x03\x04\x05"), [])
        self.assertEqual(len(parser._buffer), 0)
        self.assertEqual(parser.discarded_bytes, 5)


if __name__ == "__main__":
    unittest.main()
