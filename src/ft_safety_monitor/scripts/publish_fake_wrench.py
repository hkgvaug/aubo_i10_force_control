#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fake KWR75 wrench publisher for simulation / fake execution tests.

It never talks to hardware.  Modes can be switched at runtime by publishing
a std_msgs/String to ~command (e.g. /fake_wrench/command):

  normal
  warn_force
  stop_force
  warn_torque
  stop_torque
  dropout
  burst_force
  burst_torque

The one-shot modes return to normal after ~duration seconds.  Use
duration:=0 to keep them active until the next command.
"""

import math
import time

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

MODES = (
    "normal", "warn_force", "stop_force",
    "warn_torque", "stop_torque",
    "dropout", "burst_force", "burst_torque",
)


class FakeWrenchPublisher(object):
    def __init__(self):
        rospy.init_node("fake_wrench")

        self.topic = rospy.get_param("~topic", "/ft_sensor/wrench_raw")
        self.frame_id = rospy.get_param("~frame_id", "ft_sensor_link")
        self.rate = max(1.0, float(rospy.get_param("~rate", 250.0)))
        self.rated_force = float(rospy.get_param("~rated_force_n", 200.0))
        self.rated_torque = float(rospy.get_param("~rated_torque_nm", 8.0))
        self.duration = max(0.0, float(rospy.get_param("~duration", 2.0)))
        self.burst_on = max(1, int(rospy.get_param("~burst_on_frames", 4)))
        self.burst_off = max(1, int(rospy.get_param("~burst_off_frames", 1)))

        self.mode = str(rospy.get_param("~mode", "normal")).strip()
        if self.mode not in MODES:
            rospy.logwarn("Unknown initial mode '%s'; using normal", self.mode)
            self.mode = "normal"
        self.mode_start = time.monotonic()
        self.node_start = time.monotonic()
        self.burst_counter = 0

        self.pub = rospy.Publisher(self.topic, WrenchStamped, queue_size=10)
        self.cmd_sub = rospy.Subscriber(
            "~command", String, self._on_command, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate), self._tick)

        rospy.logwarn(
            "Fake wrench on %s at %.0f Hz, rated=(%.1f N, %.2f Nm), "
            "mode=%s. Command topic: %s",
            self.topic, self.rate, self.rated_force, self.rated_torque,
            self.mode, rospy.resolve_name("~command"))

    def _on_command(self, msg):
        mode = msg.data.strip()
        if mode not in MODES:
            rospy.logwarn("Ignoring unknown fake wrench mode '%s'", mode)
            return
        self.mode = mode
        self.mode_start = time.monotonic()
        self.burst_counter = 0
        rospy.logwarn("Fake wrench mode -> %s", mode)

    def _maybe_return_to_normal(self, now):
        if self.mode in ("normal", "burst_force", "burst_torque"):
            return
        if self.duration <= 0.0:
            return
        if now - self.mode_start < self.duration:
            return
        self.mode = "normal"
        self.mode_start = now
        self.burst_counter = 0
        rospy.logwarn("Fake wrench returned to normal")

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        now = time.monotonic()
        if self.mode == "dropout":
            if self.duration <= 0.0 or now - self.mode_start < self.duration:
                return
            self.mode = "normal"
            self.mode_start = now
            rospy.logwarn("Fake wrench dropout finished")

        self._maybe_return_to_normal(now)

        t = now - self.node_start
        fx = 2.0 * math.sin(2.0 * math.pi * 0.7 * t)
        fy = 1.5 * math.sin(2.0 * math.pi * 0.4 * t + 0.7)
        fz = 1.0 * math.sin(2.0 * math.pi * 0.5 * t + 1.2)
        mx = 0.02 * math.sin(2.0 * math.pi * 0.3 * t)
        my = 0.015 * math.sin(2.0 * math.pi * 0.25 * t + 0.5)
        mz = 0.01 * math.sin(2.0 * math.pi * 0.2 * t + 1.0)

        if self.mode == "warn_force":
            fx = 0.60 * self.rated_force
        elif self.mode == "stop_force":
            fx = 0.80 * self.rated_force
        elif self.mode == "warn_torque":
            mx = 0.60 * self.rated_torque
        elif self.mode == "stop_torque":
            mx = 0.80 * self.rated_torque
        elif self.mode == "burst_force":
            period = self.burst_on + self.burst_off
            fx = (0.80 * self.rated_force
                  if self.burst_counter < self.burst_on else 0.0)
            self.burst_counter = (self.burst_counter + 1) % period
        elif self.mode == "burst_torque":
            period = self.burst_on + self.burst_off
            mx = (0.80 * self.rated_torque
                  if self.burst_counter < self.burst_on else 0.0)
            self.burst_counter = (self.burst_counter + 1) % period

        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        msg.wrench.torque.x = mx
        msg.wrench.torque.y = my
        msg.wrench.torque.z = mz
        self.pub.publish(msg)


def main():
    FakeWrenchPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
