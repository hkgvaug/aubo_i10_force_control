#!/usr/bin/env python3
"""ROS1 Noetic driver for the KWR75-RS422 six-axis force/torque sensor.

Checklist implemented here:
  1. Send STOP, flush the input buffer, then send START; STOP again on exit.
  2. Resynchronise on 0x48/0x49 + 0xAA, require 28 bytes and 0D 0A.
  3. Publish SI units only (N and N*m).
  4. Software tare via /ft_sensor/tare (average of recent unloaded samples).
  5. Warn at 80% and error at 100% of the configured rated load.
  6. frame_id is configurable so it can match the URDF sensor link.
  7. Read the serial port in a dedicated thread, publish at a lower rate.
"""

import math
import threading
import time
from collections import deque

import rospy
import serial
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger, TriggerResponse

from kwr75_ft_sensor.protocol import Kwr75FrameParser

STOP_COMMAND = b"\x43\xaa\r\n"
START_COMMAND = b"\x48\xaa\r\n"

WRENCH_TOPIC = "/ft_sensor/wrench"
RAW_WRENCH_TOPIC = "/ft_sensor/wrench_raw"
TARE_SERVICE = "/ft_sensor/tare"


class Kwr75FtSensorNode(object):
    def __init__(self):
        self.port_name = str(rospy.get_param("~port", "/dev/robot/kwr75_force_sensor"))
        self.baudrate = int(rospy.get_param("~baudrate", 460800))
        self.publish_rate = float(rospy.get_param("~publish_rate", 250.0))
        self.frame_id = str(rospy.get_param("~frame_id", "ft_sensor_link"))
        self.raw_topic = str(rospy.get_param("~raw_topic", RAW_WRENCH_TOPIC))
        self.publish_raw = bool(rospy.get_param("~publish_raw", True))

        self.serial_timeout = float(rospy.get_param("~serial_timeout", 0.05))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 0.5))

        self.rated_force_n = float(rospy.get_param("~rated_force_n", 200.0))
        self.rated_torque_nm = float(rospy.get_param("~rated_torque_nm", 8.0))
        self.warn_ratio = float(rospy.get_param("~warn_ratio", 0.8))
        self.warn_period = float(rospy.get_param("~warn_period", 1.0))

        self.lowpass_tau = float(rospy.get_param("~lowpass_tau", 0.01))
        self.tare_frames = max(1, int(rospy.get_param("~tare_frames", 200)))
        self.tare_min_samples = max(1, int(rospy.get_param("~tare_min_samples", 20)))
        self.tare_max_std_force = float(rospy.get_param("~tare_max_std_force", 2.0))
        self.tare_max_std_torque = float(rospy.get_param("~tare_max_std_torque", 0.2))

        if not 0.0 < self.publish_rate <= 1000.0:
            raise ValueError("~publish_rate must be in (0, 1000]")
        if not 0.0 < self.warn_ratio <= 1.0:
            raise ValueError("~warn_ratio must be in (0, 1]")
        if self.lowpass_tau < 0.0:
            raise ValueError("~lowpass_tau must be >= 0 (0 disables filtering)")
        if self.rated_force_n <= 0.0 or self.rated_torque_nm <= 0.0:
            raise ValueError("rated force/torque must be positive")

        self._lock = threading.Lock()
        self._parser = Kwr75FrameParser()
        self._latest = None
        self._latest_mono = 0.0
        self._recent = deque(maxlen=self.tare_frames)
        self._bias = (0.0,) * 6

        self._filtered = None
        self._last_filter_mono = None
        self._first_frame_logged = False
        self._closing = False

        self._ser = None
        self._reader_thread = None

        self._publisher = rospy.Publisher(WRENCH_TOPIC, WrenchStamped, queue_size=1)
        # The raw topic is the low-pass filtered value WITHOUT the tare bias.
        # Safety/overload supervision must use this one: a tared value can
        # hide a large static load (tool weight, fixture preload).
        self._raw_publisher = None
        if self.publish_raw:
            self._raw_publisher = rospy.Publisher(
                self.raw_topic, WrenchStamped, queue_size=1)
        self._tare_server = rospy.Service(TARE_SERVICE, Trigger, self._handle_tare)
        rospy.on_shutdown(self._on_shutdown)

        self._open_serial()
        try:
            self._start_stream()
            self._reader_thread = threading.Thread(
                target=self._reader_loop, name="kwr75_reader", daemon=True)
            self._reader_thread.start()
        except Exception:
            self._close_serial()
            raise

        rospy.loginfo(
            "KWR75 ready: port=%s baud=%d publish=%.0f Hz frame_id=%s "
            "rated=(%.1f N, %.2f Nm) warn=%.0f%%",
            self.port_name, self.baudrate, self.publish_rate, self.frame_id,
            self.rated_force_n, self.rated_torque_nm, self.warn_ratio * 100.0)
        rospy.loginfo(
            "KWR75 topics: tared=%s raw=%s (publish_raw=%s)",
            WRENCH_TOPIC, self.raw_topic if self.publish_raw else "disabled",
            self.publish_raw)

    # ------------------------------------------------------------------ serial

    def _open_serial(self):
        try:
            self._ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.serial_timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        except serial.SerialException as exc:
            rospy.logfatal("Cannot open KWR75 serial port %s: %s", self.port_name, exc)
            raise

        rospy.loginfo("Opened %s at %d baud, 8N1", self.port_name, self.baudrate)

    def _start_stream(self):
        # Stop first so no half frame is left over from a previous run, then
        # discard whatever the driver buffered before asking for a new stream.
        self._ser.write(STOP_COMMAND)
        self._ser.flush()
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        self._ser.write(START_COMMAND)
        self._ser.flush()
        rospy.loginfo("Sent KWR75 stop/start sequence (0x43... then 0x48...)")

    def _close_serial(self):
        serial_port, self._ser = self._ser, None
        if serial_port is None:
            return
        try:
            if serial_port.is_open:
                serial_port.write(STOP_COMMAND)
                serial_port.flush()
                time.sleep(0.02)
        except (serial.SerialException, OSError) as exc:
            rospy.logwarn("Could not send KWR75 stop command: %s", exc)
        finally:
            try:
                serial_port.close()
            except Exception:
                pass

    def _reader_loop(self):
        while not rospy.is_shutdown() and not self._closing:
            serial_port = self._ser
            if serial_port is None:
                return
            try:
                waiting = serial_port.in_waiting
                chunk = serial_port.read(waiting if waiting > 0 else 1)
            except (serial.SerialException, OSError) as exc:
                if not self._closing and not rospy.is_shutdown():
                    rospy.logerr("KWR75 serial read failed: %s", exc)
                    rospy.signal_shutdown("KWR75 serial port error")
                return

            if not chunk:
                continue

            for sample in self._parser.feed(chunk):
                self._handle_sample(sample)

    def _handle_sample(self, sample):
        values = (sample.fx, sample.fy, sample.fz,
                  sample.mx, sample.my, sample.mz)
        now = time.monotonic()

        with self._lock:
            self._latest = values
            self._latest_mono = now
            self._recent.append(values)

        if not self._first_frame_logged:
            self._first_frame_logged = True
            rospy.loginfo(
                "First KWR75 frame received (header=0x%02X): "
                "F=(%.2f, %.2f, %.2f) N, M=(%.3f, %.3f, %.3f) Nm",
                sample.header, *values)

    def _on_shutdown(self):
        self._closing = True
        self._close_serial()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            if threading.current_thread() is not self._reader_thread:
                self._reader_thread.join(timeout=0.5)

        parser = self._parser
        rospy.loginfo(
            "KWR75 stopped: frames=%d bad_tail=%d bad_payload=%d discarded_bytes=%d",
            parser.frames_ok, parser.bad_terminator,
            parser.bad_payload, parser.discarded_bytes)

    # ----------------------------------------------------------------- publish

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self._publish_once()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _publish_once(self):
        now = time.monotonic()
        with self._lock:
            values = self._latest
            values_mono = self._latest_mono
            bias = self._bias

        if values is None:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for the first KWR75 frame. Check power, wiring and "
                "that the converter can transmit (sensor needs 0x48 AA 0D 0A).")
            return

        if now - values_mono > self.stale_timeout:
            rospy.logwarn_throttle(
                1.0,
                "No valid KWR75 frame for %.2f s; check serial cabling/baud",
                now - values_mono)
            return

        if self.lowpass_tau > 0.0:
            if self._filtered is None or self._last_filter_mono is None:
                self._filtered = values
            else:
                dt = max(now - self._last_filter_mono, 1e-6)
                alpha = 1.0 - math.exp(-dt / self.lowpass_tau)
                self._filtered = tuple(
                    previous + alpha * (value - previous)
                    for previous, value in zip(self._filtered, values))
            self._last_filter_mono = now
            filtered = self._filtered
        else:
            filtered = values

        if self._raw_publisher is not None:
            # Raw = filtered, no tare bias.  This is what the overload
            # monitor must use.
            self._raw_publisher.publish(self._build_message(filtered))

        output = tuple(value - offset for value, offset in zip(filtered, bias))
        self._publisher.publish(self._build_message(output))

        self._check_rated_load(filtered)

    def _build_message(self, values):
        message = WrenchStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.wrench.force.x = values[0]
        message.wrench.force.y = values[1]
        message.wrench.force.z = values[2]
        message.wrench.torque.x = values[3]
        message.wrench.torque.y = values[4]
        message.wrench.torque.z = values[5]
        return message

    def _check_rated_load(self, values):
        # Use the un-tared, filtered values so a bad tare cannot hide a real load.
        force_peak = max(abs(value) for value in values[0:3])
        torque_peak = max(abs(value) for value in values[3:6])

        if force_peak >= self.rated_force_n or torque_peak >= self.rated_torque_nm:
            rospy.logerr_throttle(
                self.warn_period,
                "KWR75 OVERLOAD: |F|=%.1f N (rated %.1f N), "
                "|M|=%.2f Nm (rated %.2f Nm). Unload immediately; long-term "
                "overload can permanently damage the sensor.",
                force_peak, self.rated_force_n,
                torque_peak, self.rated_torque_nm)
            return

        force_warn = self.rated_force_n * self.warn_ratio
        torque_warn = self.rated_torque_nm * self.warn_ratio
        if force_peak >= force_warn or torque_peak >= torque_warn:
            rospy.logwarn_throttle(
                self.warn_period,
                "KWR75 load above %.0f%%: |F|=%.1f N (warn at %.1f N), "
                "|M|=%.2f Nm (warn at %.2f Nm)",
                self.warn_ratio * 100.0, force_peak, force_warn,
                torque_peak, torque_warn)

    # ------------------------------------------------------------------- tare

    def _handle_tare(self, _request):
        now = time.monotonic()
        with self._lock:
            recent = list(self._recent)
            latest_mono = self._latest_mono

        if not recent or now - latest_mono > self.stale_timeout:
            message = "No fresh KWR75 samples; tare not applied"
            rospy.logwarn(message)
            return TriggerResponse(False, message)

        if len(recent) < self.tare_min_samples:
            message = ("Only %d samples available, need at least %d; "
                       "tare not applied" % (len(recent), self.tare_min_samples))
            rospy.logwarn(message)
            return TriggerResponse(False, message)

        means = [
            sum(sample[axis] for sample in recent) / len(recent)
            for axis in range(6)
        ]
        stds = [
            math.sqrt(sum((sample[axis] - means[axis]) ** 2
                          for sample in recent) / len(recent))
            for axis in range(6)
        ]
        std_force = max(stds[0:3])
        std_torque = max(stds[3:6])

        if std_force > self.tare_max_std_force or std_torque > self.tare_max_std_torque:
            message = (
                "Samples are not stable (std F=%.2f N, M=%.3f Nm); keep the "
                "sensor unloaded and still, then tare again"
                % (std_force, std_torque))
            rospy.logwarn(message)
            return TriggerResponse(False, message)

        with self._lock:
            self._bias = tuple(means)

        rospy.loginfo(
            "Tare applied over %d samples: F=(%.3f, %.3f, %.3f) N, "
            "M=(%.3f, %.3f, %.3f) Nm",
            len(recent), *means)
        return TriggerResponse(
            True,
            ("Tared over %d samples: F=(%.3f, %.3f, %.3f) N, "
             "M=(%.3f, %.3f, %.3f) Nm" % (len(recent), *means)))


def main():
    rospy.init_node("kwr75_ft_sensor")
    try:
        node = Kwr75FtSensorNode()
    except rospy.ROSInterruptException:
        return
    except Exception as exc:
        rospy.logfatal("KWR75 node failed to start: %s", exc)
        return
    node.spin()


if __name__ == "__main__":
    main()
