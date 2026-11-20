#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KWR75 force/torque safety monitor.

AUXILIARY SOFTWARE PROTECTION ONLY.
This node is not safety-rated and must never be the only protection for
people or equipment.  The real protection is the robot emergency stop /
safety PLC.  Read docs/stop_path_analysis.md before using it on hardware.

Behaviour:
  * subscribes /ft_sensor/wrench
  * warns at a configurable fraction of the rated force/torque
  * faults at a configurable fraction, after N consecutive frames
  * faults if no sensor frame arrives for the watchdog timeout
  * faults are latched until /ft_safety/reset is called and the wrench is
    back below the warning threshold with fresh data
  * on a fault it publishes /trajectory_execution_event "stop" (this is
    what clears the aubo_driver queue), publishes
    /aubo_driver/cancel_trajectory 1, calls MoveGroupCommander.stop() and
    cancels the follow_joint_trajectory goal
"""

import json
import math
import sys
import threading
import time

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger, TriggerResponse

try:
    import moveit_commander
except ImportError:  # pragma: no cover - depends on the installed MoveIt
    moveit_commander = None

STATE_STARTING = "STARTING"
STATE_NORMAL = "NORMAL"
STATE_WARNING = "WARNING"
STATE_FAULT = "FAULT"

AXIS_NAMES = ("fx", "fy", "fz", "mx", "my", "mz")


def _peak(values, indices):
    """Return (largest absolute value, axis name) over the given indices."""
    best = 0.0
    best_name = "none"
    for index in indices:
        value = abs(values[index])
        if value > best:
            best = value
            best_name = AXIS_NAMES[index]
    return best, best_name


class FtSafetyMonitor(object):
    def __init__(self):
        rospy.init_node("ft_safety_monitor", anonymous=False)

        # ------------------------------------------------------------ params
        # Raw = un-tared.  Overload protection must see the absolute load;
        # the tared topic would hide tool weight / fixture preload.
        self.wrench_topic = rospy.get_param(
            "~wrench_topic", "/ft_sensor/wrench_raw")
        self.state_topic = rospy.get_param("~state_topic", "/ft_safety/state")
        self.reset_service = rospy.get_param("~reset_service", "/ft_safety/reset")
        self.expected_frame_id = rospy.get_param("~expected_frame_id", "ft_sensor_link")

        self.rated_force = float(rospy.get_param("~rated_force_n", 200.0))
        self.rated_torque = float(rospy.get_param("~rated_torque_nm", 8.0))
        if self.rated_force <= 0.0 or self.rated_torque <= 0.0:
            raise ValueError("rated_force_n and rated_torque_nm must be positive")

        self.warn_ratio_force = float(rospy.get_param("~warn_ratio_force", 0.50))
        self.warn_ratio_torque = float(rospy.get_param("~warn_ratio_torque", 0.50))
        self.stop_ratio_force = float(rospy.get_param("~stop_ratio_force", 0.75))
        self.stop_ratio_torque = float(rospy.get_param("~stop_ratio_torque", 0.75))
        for name, value in (
                ("warn_ratio_force", self.warn_ratio_force),
                ("warn_ratio_torque", self.warn_ratio_torque),
                ("stop_ratio_force", self.stop_ratio_force),
                ("stop_ratio_torque", self.stop_ratio_torque)):
            if not 0.0 < value <= 1.0:
                raise ValueError("%s must be in (0, 1]" % name)
        if (self.stop_ratio_force <= self.warn_ratio_force
                or self.stop_ratio_torque <= self.warn_ratio_torque):
            rospy.logwarn(
                "Stop ratio is not above the warning ratio; both conditions "
                "will still be evaluated independently")

        self.warn_force = self.rated_force * self.warn_ratio_force
        self.warn_torque = self.rated_torque * self.warn_ratio_torque
        self.stop_force = self.rated_force * self.stop_ratio_force
        self.stop_torque = self.rated_torque * self.stop_ratio_torque

        self.debounce_frames = max(1, int(rospy.get_param("~debounce_frames", 5)))
        self.warn_clear_frames = max(1, int(rospy.get_param("~warn_clear_frames", 25)))
        self.watchdog_timeout = max(0.01, float(rospy.get_param("~watchdog_timeout", 0.20)))
        self.startup_grace = max(0.0, float(rospy.get_param("~startup_grace_period", 2.0)))
        self.publish_rate = max(1.0, float(rospy.get_param("~publish_rate", 20.0)))
        # Collect the first N raw frames and log the static baseline before
        # the warn/stop state machine starts.  This makes "tool weight is
        # already above the warning threshold" distinguishable from a real
        # contact/fault.
        self.baseline_frames = max(0, int(rospy.get_param("~baseline_frames", 50)))

        self.use_driver_stop = bool(rospy.get_param("~use_driver_stop", True))
        self.use_cancel_trajectory = bool(rospy.get_param("~use_cancel_trajectory", True))
        self.use_move_group_stop = bool(rospy.get_param("~use_move_group_stop", True))
        self.use_action_cancel = bool(rospy.get_param("~use_action_cancel", True))

        self.group_name = rospy.get_param("~group_name", "manipulator_i10")
        self.action_namespace = rospy.get_param(
            "~action_namespace", "/aubo_i10_controller/follow_joint_trajectory")
        self.trajectory_event_topic = rospy.get_param(
            "~trajectory_event_topic", "/trajectory_execution_event")
        self.cancel_trajectory_topic = rospy.get_param(
            "~cancel_trajectory_topic", "/aubo_driver/cancel_trajectory")

        self.stop_repeat_count = max(1, int(rospy.get_param("~stop_repeat_count", 3)))
        self.stop_repeat_period = max(0.0, float(rospy.get_param("~stop_repeat_period", 0.05)))
        self.action_server_timeout = max(
            0.0, float(rospy.get_param("~action_server_timeout", 1.0)))

        # ------------------------------------------------------------- state
        self._lock = threading.Lock()
        self._state = STATE_STARTING
        self._reason = "waiting for first sensor frame"
        self._latched = False
        self._start_mono = time.monotonic()
        self._fault_mono = None
        self._last_msg_mono = None
        self._last_msg_wall = None
        self._values = None
        self._last_force_peak = 0.0
        self._last_torque_peak = 0.0
        self._last_force_axis = "none"
        self._last_torque_axis = "none"
        self._warn_count = 0
        self._stop_count = 0
        self._clear_count = 0
        self._stop_started = False
        self._baseline_samples = []
        self._baseline_done = self.baseline_frames == 0
        self._baseline = None

        self._mg_lock = threading.Lock()
        self._mg_init_started = False
        self._move_group = None

        # --------------------------------------------------------- interfaces
        self.state_pub = rospy.Publisher(
            self.state_topic, String, queue_size=10, latch=True)
        self.event_pub = rospy.Publisher(
            self.trajectory_event_topic, String, queue_size=10)
        self.cancel_pub = rospy.Publisher(
            self.cancel_trajectory_topic, UInt8, queue_size=10)

        self.wrench_sub = rospy.Subscriber(
            self.wrench_topic, WrenchStamped, self._on_wrench, queue_size=10)
        self.reset_srv = rospy.Service(
            self.reset_service, Trigger, self._on_reset)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._tick)

        rospy.logwarn(
            "FT safety monitor started (AUXILIARY protection, NOT safety-rated). "
            "wrench=%s state=%s reset=%s", self.wrench_topic,
            self.state_topic, self.reset_service)
        rospy.logwarn(
            "  rated=(%.1f N, %.2f Nm) warn=(%.1f N, %.2f Nm) "
            "stop=(%.1f N, %.2f Nm) debounce=%d frames watchdog=%.0f ms",
            self.rated_force, self.rated_torque,
            self.warn_force, self.warn_torque,
            self.stop_force, self.stop_torque,
            self.debounce_frames, self.watchdog_timeout * 1000.0)

        self._publish_state()

    # ------------------------------------------------------------------ input
    def _on_wrench(self, msg):
        if msg.header.frame_id and self.expected_frame_id:
            if msg.header.frame_id != self.expected_frame_id:
                rospy.logwarn_throttle(
                    5.0, "Wrench frame_id is '%s', expected '%s'",
                    msg.header.frame_id, self.expected_frame_id)

        values = (
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        )
        now = time.monotonic()

        if not all(math.isfinite(value) for value in values):
            with self._lock:
                self._values = None
                self._last_msg_mono = now
                self._last_msg_wall = rospy.Time.now()
            self._enter_fault("non-finite values from the sensor", 0.0, 0.0)
            return

        force_peak, force_axis = _peak(values, (0, 1, 2))
        torque_peak, torque_axis = _peak(values, (3, 4, 5))
        fault_reason = None
        state_changed = False
        old_state = None
        collecting_baseline = False

        with self._lock:
            self._values = values
            self._last_msg_mono = now
            self._last_msg_wall = rospy.Time.now()
            self._last_force_peak = force_peak
            self._last_torque_peak = torque_peak
            self._last_force_axis = force_axis
            self._last_torque_axis = torque_axis

            if not self._baseline_done:
                self._baseline_samples.append(values)
                if len(self._baseline_samples) >= self.baseline_frames:
                    self._baseline = self._compute_baseline(
                        self._baseline_samples)
                    self._baseline_done = True
                    # One-time log; must appear before the state machine
                    # can latch a FAULT caused by the same static load.
                    self._log_baseline()
                else:
                    collecting_baseline = True

            if not collecting_baseline and not self._latched:
                if self._state == STATE_STARTING:
                    self._state = STATE_NORMAL
                    self._reason = "sensor data received"
                    state_changed = True
                    old_state = STATE_STARTING

                stop_hit = (force_peak >= self.stop_force
                            or torque_peak >= self.stop_torque)
                warn_hit = (force_peak >= self.warn_force
                            or torque_peak >= self.warn_torque)

                self._stop_count = self._stop_count + 1 if stop_hit else 0
                if warn_hit:
                    self._warn_count += 1
                    self._clear_count = 0
                else:
                    self._warn_count = 0
                    self._clear_count += 1

                if self._stop_count >= self.debounce_frames:
                    fault_reason = (
                        "wrench above stop threshold for %d frames "
                        "(force %.3f N on %s, limit %.3f N; "
                        "torque %.3f Nm on %s, limit %.3f Nm)"
                        % (self._stop_count, force_peak, force_axis,
                           self.stop_force, torque_peak, torque_axis,
                           self.stop_torque))
                elif (self._warn_count >= self.debounce_frames
                      and self._state != STATE_WARNING):
                    old_state = self._state
                    self._state = STATE_WARNING
                    self._reason = (
                        "wrench above warning threshold for %d frames"
                        % self._warn_count)
                    state_changed = True
                elif (self._state == STATE_WARNING
                      and self._clear_count >= self.warn_clear_frames):
                    old_state = self._state
                    self._state = STATE_NORMAL
                    self._reason = "wrench back below warning threshold"
                    state_changed = True

        if collecting_baseline:
            self._publish_state()
            return

        if fault_reason:
            self._enter_fault(fault_reason, force_peak, torque_peak)
        elif state_changed:
            if self._state == STATE_WARNING:
                rospy.logwarn("FT safety %s -> %s: %s",
                              old_state, self._state, self._reason)
            else:
                rospy.loginfo("FT safety %s -> %s: %s",
                              old_state, self._state, self._reason)
            self._publish_state()

    def _compute_baseline(self, samples):
        count = len(samples)
        means = [sum(sample[i] for sample in samples) / float(count)
                 for i in range(6)]
        peaks = [max(abs(sample[i]) for sample in samples)
                 for i in range(6)]
        force_peak, force_axis = _peak(peaks, (0, 1, 2))
        torque_peak, torque_axis = _peak(peaks, (3, 4, 5))
        return {
            "frames": count,
            "force_mean": {"x": means[0], "y": means[1], "z": means[2]},
            "torque_mean": {"x": means[3], "y": means[4], "z": means[5]},
            "force_peak": force_peak,
            "force_peak_axis": force_axis,
            "torque_peak": torque_peak,
            "torque_peak_axis": torque_axis,
            "above_warn": bool(force_peak >= self.warn_force
                               or torque_peak >= self.warn_torque),
            "above_stop": bool(force_peak >= self.stop_force
                               or torque_peak >= self.stop_torque),
        }

    def _log_baseline(self):
        baseline = self._baseline
        if baseline is None:
            return
        force_mean = baseline["force_mean"]
        torque_mean = baseline["torque_mean"]
        common = (
            baseline["frames"],
            force_mean["x"], force_mean["y"], force_mean["z"],
            baseline["force_peak"], baseline["force_peak_axis"],
            torque_mean["x"], torque_mean["y"], torque_mean["z"],
            baseline["torque_peak"], baseline["torque_peak_axis"],
            self.warn_force, self.warn_torque,
            self.stop_force, self.stop_torque,
        )
        text = (
            "force mean (%.3f, %.3f, %.3f) N, peak %.3f N on %s; "
            "torque mean (%.3f, %.3f, %.3f) Nm, peak %.3f Nm on %s "
            "(warn %.3f N / %.3f Nm, stop %.3f N / %.3f Nm)")
        if baseline["above_stop"]:
            rospy.logerr(
                "FT startup baseline over %d frames EXCEEDS THE STOP "
                "THRESHOLD: " + text + ". This is a STATIC LOAD "
                "(tool weight / fixture preload), NOT a sensor or contact "
                "fault. The monitor will enter FAULT. Do gravity "
                "compensation or raise the thresholds before use.", *common)
        elif baseline["above_warn"]:
            rospy.logwarn(
                "FT startup baseline over %d frames EXCEEDS THE WARNING "
                "THRESHOLD: " + text + ". This is a STATIC LOAD "
                "(tool weight / fixture preload), NOT a sensor fault. "
                "The monitor will enter WARNING.", *common)
        else:
            rospy.loginfo(
                "FT startup baseline over %d frames: " + text + ". "
                "Below the warning thresholds.", *common)

    # ----------------------------------------------------------------- output
    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        now = time.monotonic()
        fault = None
        with self._lock:
            if not self._latched:
                if self._last_msg_mono is None:
                    if now - self._start_mono > self.startup_grace:
                        fault = (
                            "no FT data since startup (grace %.0f ms)"
                            % (self.startup_grace * 1000.0))
                else:
                    age = now - self._last_msg_mono
                    if age > self.watchdog_timeout:
                        fault = (
                            "FT watchdog timeout: no data for %.0f ms "
                            "(limit %.0f ms)"
                            % (age * 1000.0, self.watchdog_timeout * 1000.0))
        if fault:
            self._enter_fault(fault)
        self._publish_state()

    def _publish_state(self):
        if rospy.is_shutdown():
            return
        now = time.monotonic()
        with self._lock:
            values = self._values
            last_mono = self._last_msg_mono
            payload = {
                "state": self._state,
                "latched": self._latched,
                "reason": self._reason,
                "stamp": rospy.Time.now().to_sec(),
                "sensor_age": (None if last_mono is None
                               else round(now - last_mono, 4)),
                "force_peak": self._last_force_peak,
                "force_axis": self._last_force_axis,
                "torque_peak": self._last_torque_peak,
                "torque_axis": self._last_torque_axis,
                "counters": {
                    "warn": self._warn_count,
                    "stop": self._stop_count,
                    "clear": self._clear_count,
                },
                "thresholds": {
                    "force_warn": self.warn_force,
                    "force_stop": self.stop_force,
                    "torque_warn": self.warn_torque,
                    "torque_stop": self.stop_torque,
                },
                "rated": {
                    "force": self.rated_force,
                    "torque": self.rated_torque,
                },
                "baseline": self._baseline,
            }
            if values is None:
                payload["force"] = None
                payload["torque"] = None
            else:
                payload["force"] = {
                    "x": values[0], "y": values[1], "z": values[2]}
                payload["torque"] = {
                    "x": values[3], "y": values[4], "z": values[5]}

        try:
            self.state_pub.publish(
                String(data=json.dumps(payload, allow_nan=False)))
        except (TypeError, ValueError) as exc:
            rospy.logerr_throttle(5.0, "Cannot serialize safety state: %s", exc)
        except rospy.ROSException as exc:
            # Expected during shutdown; do not let the timer thread die noisily.
            rospy.logdebug_throttle(5.0, "State publisher closed: %s", exc)

    # ------------------------------------------------------------------ fault
    def _enter_fault(self, reason, force_peak=None, torque_peak=None):
        with self._lock:
            if self._latched:
                return
            self._latched = True
            self._state = STATE_FAULT
            self._reason = reason
            self._fault_mono = time.monotonic()
            if force_peak is not None:
                self._last_force_peak = force_peak
            if torque_peak is not None:
                self._last_torque_peak = torque_peak

        rospy.logerr(
            "FT SAFETY FAULT (latched): %s. "
            "This is auxiliary protection, not a safety-rated stop.", reason)
        self._publish_state()

        # Synchronous first shot.  Creating MoveGroupCommander can hold the
        # GIL for ~1-2 s, so it must not delay the driver stop event.
        if self.use_driver_stop:
            try:
                self.event_pub.publish(String(data="stop"))
                if self.use_cancel_trajectory:
                    self.cancel_pub.publish(UInt8(data=1))
            except Exception as exc:  # noqa: BLE001
                rospy.logerr("Failed to publish the first stop event: %s", exc)

        with self._lock:
            if self._stop_started:
                return
            self._stop_started = True

        threading.Thread(
            target=self._driver_stop_worker, args=(reason,),
            name="ft_safety_driver_stop", daemon=True).start()
        threading.Thread(
            target=self._action_cancel_worker,
            name="ft_safety_action_cancel", daemon=True).start()
        threading.Thread(
            target=self._move_group_stop_worker,
            name="ft_safety_move_group_stop", daemon=True).start()

    def _driver_stop_worker(self, reason):
        if not self.use_driver_stop:
            return
        for index in range(self.stop_repeat_count):
            if rospy.is_shutdown():
                return
            try:
                self.event_pub.publish(String(data="stop"))
                if self.use_cancel_trajectory:
                    self.cancel_pub.publish(UInt8(data=1))
            except Exception as exc:  # noqa: BLE001 - keep the worker alive
                rospy.logerr("Failed to publish stop event: %s", exc)
            if index + 1 < self.stop_repeat_count:
                rospy.sleep(self.stop_repeat_period)
        rospy.logwarn(
            "Published stop on %s and %s (%d times): %s",
            self.trajectory_event_topic,
            self.cancel_trajectory_topic,
            self.stop_repeat_count, reason)

    def _action_cancel_worker(self):
        if not self.use_action_cancel:
            return
        try:
            client = actionlib.SimpleActionClient(
                self.action_namespace, FollowJointTrajectoryAction)
            if client.wait_for_server(rospy.Duration(self.action_server_timeout)):
                client.cancel_all_goals()
                rospy.logwarn(
                    "Canceled follow_joint_trajectory goal(s) on %s",
                    self.action_namespace)
            else:
                rospy.logwarn(
                    "Action server %s is not available; action cancel skipped",
                    self.action_namespace)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("Action cancel failed: %s", exc)

    def _move_group_stop_worker(self):
        if not self.use_move_group_stop:
            return
        if moveit_commander is None:
            rospy.logwarn(
                "moveit_commander is not available; MoveGroupCommander.stop() "
                "was skipped (the driver stop event was still published)")
            return
        try:
            with self._mg_lock:
                if not self._mg_init_started:
                    self._mg_init_started = True
                    rospy.logwarn(
                        "Creating MoveGroupCommander(%s) to call stop(). "
                        "This may block until move_group is available.",
                        self.group_name)
                    self._move_group = moveit_commander.MoveGroupCommander(
                        self.group_name)
                move_group = self._move_group
            if move_group is None:
                rospy.logwarn(
                    "MoveGroupCommander is still being created; "
                    "MoveGroupCommander.stop() skipped this time")
                return
            move_group.stop()
            rospy.logwarn("MoveGroupCommander.stop() called")
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("MoveGroupCommander.stop() failed: %s", exc)

    # ------------------------------------------------------------------ reset
    def _on_reset(self, _request):
        now = time.monotonic()
        with self._lock:
            latched = self._latched
            values = self._values
            last_mono = self._last_msg_mono
            warn_force = self.warn_force
            warn_torque = self.warn_torque

        if not latched:
            return TriggerResponse(False, "monitor is not in FAULT")

        if (values is None or last_mono is None
                or now - last_mono > self.watchdog_timeout):
            return TriggerResponse(
                False,
                "sensor data is stale; cannot confirm that the wrench is safe")

        force_peak, force_axis = _peak(values, (0, 1, 2))
        torque_peak, torque_axis = _peak(values, (3, 4, 5))
        if force_peak >= warn_force or torque_peak >= warn_torque:
            return TriggerResponse(
                False,
                "wrench is still above the warning threshold "
                "(force %.3f N on %s / %.3f N, torque %.3f Nm on %s / %.3f Nm)"
                % (force_peak, force_axis, warn_force,
                   torque_peak, torque_axis, warn_torque))

        with self._lock:
            if not self._latched:
                return TriggerResponse(False, "fault was already cleared")
            self._latched = False
            self._state = STATE_NORMAL
            self._reason = "manual reset"
            self._warn_count = 0
            self._stop_count = 0
            self._clear_count = 0
            self._stop_started = False

        rospy.logwarn(
            "FT safety fault cleared by service call. "
            "Robot motion is NOT restarted automatically.")
        self._publish_state()
        return TriggerResponse(
            True,
            "fault cleared; motion is not restarted automatically")


def main():
    if moveit_commander is not None:
        try:
            moveit_commander.roscpp_initialize(sys.argv)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("moveit_commander.roscpp_initialize failed: %s", exc)

    try:
        FtSafetyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if moveit_commander is not None:
            try:
                moveit_commander.roscpp_shutdown()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
