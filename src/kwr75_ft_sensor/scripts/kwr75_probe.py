#!/usr/bin/env python3
"""Standalone KWR75-RS422 probe. Does not need ROS.

Use this before starting the ROS node to verify power, wiring, the
RS422/USB converter's transmit direction and the 460800 8N1 setting.

Example:
    python3 kwr75_probe.py --port /dev/ttyUSB0 --duration 10 --raw
"""

import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: sudo apt install python3-serial")

try:
    from kwr75_ft_sensor.protocol import Kwr75FrameParser
except ImportError:
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from kwr75_ft_sensor.protocol import Kwr75FrameParser

STOP_COMMAND = b"\x43\xaa\r\n"
START_COMMAND = b"\x48\xaa\r\n"


def parse_args():
    parser = argparse.ArgumentParser(description="KWR75-RS422 serial probe")
    parser.add_argument("--port", required=True,
                        help="serial device, e.g. /dev/ttyUSB0 or /dev/robot/kwr75_force_sensor")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--duration", type=float, default=10.0,
                        help="seconds to probe (default: 10)")
    parser.add_argument("--raw", action="store_true",
                        help="print the first five raw frames as hex")
    parser.add_argument("--report", type=float, default=0.5,
                        help="summary period in seconds (default: 0.5)")
    return parser.parse_args()


def main():
    args = parse_args()
    parser = Kwr75FrameParser()

    try:
        serial_port = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except serial.SerialException as exc:
        sys.exit("Cannot open %s: %s" % (args.port, exc))

    frames = 0
    raw_printed = 0
    latest = None
    started = time.monotonic()
    last_report = started

    try:
        serial_port.write(STOP_COMMAND)
        serial_port.flush()
        time.sleep(0.05)
        serial_port.reset_input_buffer()
        serial_port.write(START_COMMAND)
        serial_port.flush()
        print("Opened %s @ %d 8N1; streaming. Ctrl+C to stop." % (args.port, args.baud))

        while time.monotonic() - started < args.duration:
            chunk = serial_port.read(serial_port.in_waiting or 1)
            if not chunk:
                continue

            for sample in parser.feed(chunk):
                frames += 1
                latest = sample
                if args.raw and raw_printed < 5:
                    raw_printed += 1
                    print("raw[%d]: %s" % (raw_printed, sample.raw.hex(" ").upper()))

            now = time.monotonic()
            if now - last_report >= args.report:
                elapsed = max(now - started, 1e-6)
                if latest is None:
                    print("frames=0  elapsed=%.1fs  (no valid frame: check wiring, "
                          "power, TX direction, baud)" % elapsed)
                else:
                    print(
                        "frames=%d  rate=%.1f Hz  F=(%7.2f %7.2f %7.2f) N  "
                        "M=(%7.3f %7.3f %7.3f) Nm"
                        % (frames, frames / elapsed,
                           latest.fx, latest.fy, latest.fz,
                           latest.mx, latest.my, latest.mz))
                last_report = now

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            serial_port.write(STOP_COMMAND)
            serial_port.flush()
        except Exception:
            pass
        serial_port.close()
        print(
            "Stopped. frames=%d bad_tail=%d bad_payload=%d discarded_bytes=%d"
            % (parser.frames_ok, parser.bad_terminator,
               parser.bad_payload, parser.discarded_bytes))


if __name__ == "__main__":
    main()
