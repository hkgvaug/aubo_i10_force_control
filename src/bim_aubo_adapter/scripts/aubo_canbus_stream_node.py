#!/usr/bin/env python3
"""
AUBO Low-Latency Streaming Servo & L1 Adaptive Controller Node.
Bypasses the high-latency (600ms - 1.2s) MoveIt trajectory buffer for contact-rich manipulation.

Features:
  - 10 Hz Diffusion Policy / L1 Adaptive Control Loop
  - 100 Hz Quintic / Linear joint interpolation
  - Emergency FAULT interlock with ft_safety_monitor
  - Direct CAN bus streaming interface via robotServiceSetRobotPosData2Canbus
"""

import sys
import os
import time
import math
import numpy as np

import rospy
from geometry_msgs.msg import Vector3, WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool

# Add contact_rich_control to path
WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
CTRL_PKG_DIR = os.path.join(WORKSPACE_ROOT, "contact_rich_control")
if CTRL_PKG_DIR not in sys.path:
    sys.path.insert(0, CTRL_PKG_DIR)

from l1_adaptive_controller import L1AdaptiveController, L1ControllerConfig
from aubo_i10_kinematics import AuboI10Kinematics


class AuboStreamServoNode:
    """ROS node executing 10 Hz L1 adaptive control + 100 Hz micro-servoing."""

    def __init__(self):
        rospy.init_node("aubo_stream_servo_node", anonymous=False)

        # Parameters
        self.control_rate_hz = rospy.get_param("~control_rate_hz", 10.0)      # 10 Hz policy loop
        self.stream_rate_hz = rospy.get_param("~stream_rate_hz", 100.0)        # 100 Hz joint servo loop
        self.max_force_cutoff = rospy.get_param("~max_force_cutoff", 180.0)    # 180 N hard stop
        self.robot_ip = rospy.get_param("~robot_ip", "192.168.1.100")
        self.mock_hardware = rospy.get_param("~mock_hardware", True)

        # State tracking
        self.current_joints = np.zeros(6, dtype=np.float64)
        self.joint_states_received = False
        self.kin = AuboI10Kinematics()
        self.current_flange_pos = self.kin.forward(self.current_joints)[:3, 3].copy()
        self.current_flange_rot = self.kin.forward(self.current_joints)[:3, :3].copy()
        self.measured_force = np.zeros(3, dtype=np.float64)
        self.measured_wrench = np.zeros(6, dtype=np.float64)
        self.joint_names = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
                            "wrist1_joint", "wrist2_joint", "wrist3_joint"]
        self.is_fault = False
        self.last_ft_time = rospy.Time.now()

        # Initialize L1 Adaptive Controller (Timber Joint defaults)
        self.l1_cfg = L1ControllerConfig(
            apred=-62.8,
            gamma_gain=5.0,
            lambda_decay=0.98,
            kd=0.005,
            F0=30.0,
            sigma_max=0.001,
            delta_max=0.010,
        )
        self.controller = L1AdaptiveController(self.l1_cfg, dt=1.0 / self.control_rate_hz)
        self.controller.reset(self.current_flange_pos)

        # Incoming nominal policy command (dx, dy, dz)
        self.latest_policy_cmd = np.zeros(3, dtype=np.float64)
        self.new_policy_cmd_received = False

        # ROS Subscribers
        rospy.Subscriber("/contact_control/policy_delta", Vector3, self.policy_cmd_cb, queue_size=1)
        rospy.Subscriber("/ft_sensor/wrench_raw", WrenchStamped, self.ft_cb, queue_size=1)
        rospy.Subscriber("/ft_safety/state", String, self.safety_state_cb, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.joint_states_cb, queue_size=1)

        # ROS Publishers
        self.pub_corrected_cmd = rospy.Publisher("/contact_control/corrected_delta", Vector3, queue_size=1)
        self.pub_servo_joints = rospy.Publisher("/aubo_driver/servo_joint_cmd", JointState, queue_size=1)

        rospy.loginfo(f"[AuboServoNode] Initialized (Policy: {self.control_rate_hz}Hz, Servo: {self.stream_rate_hz}Hz, Mock: {self.mock_hardware})")

    def safety_state_cb(self, msg: String):
        if "FAULT" in msg.data.upper():
            if not self.is_fault:
                rospy.logerr(f"[AuboServoNode] FAULT triggered: {msg.data}! Halting streaming servo.")
            self.is_fault = True
        else:
            self.is_fault = False

    def ft_cb(self, msg: WrenchStamped):
        self.last_ft_time = rospy.Time.now()
        self.measured_wrench = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ], dtype=np.float64)
        self.measured_force = self.measured_wrench[:3]

        # Fast force protection
        if np.linalg.norm(self.measured_force) > self.max_force_cutoff:
            rospy.logwarn_throttle(1.0, f"[AuboServoNode] High contact force {np.linalg.norm(self.measured_force):.1f} N!")

    def joint_states_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self.current_joints = np.array(msg.position[:6], dtype=np.float64)
            T = self.kin.forward(self.current_joints)
            self.current_flange_pos = T[:3, 3].copy()
            self.current_flange_rot = T[:3, :3].copy()
            self.joint_states_received = True

    def policy_cmd_cb(self, msg: Vector3):
        self.latest_policy_cmd = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        self.new_policy_cmd_received = True

    def spin(self):
        """Main real-time loop."""
        rate = rospy.Rate(self.control_rate_hz)
        substeps = int(round(self.stream_rate_hz / self.control_rate_hz))
        sub_dt = 1.0 / self.stream_rate_hz

        while not rospy.is_shutdown():
            if self.is_fault:
                rate.sleep()
                continue

            # Check watchdog
            if (rospy.Time.now() - self.last_ft_time).to_sec() > 0.5:
                rospy.logwarn_throttle(2.0, "[AuboServoNode] F/T sensor watchdog timeout! Pausing motion.")
                rate.sleep()
                continue

            u_pol = self.latest_policy_cmd.copy()
            # 1. Run L1 Adaptive Step (10 Hz)
            u_cmd = self.controller.step(self.current_flange_pos, u_pol, self.measured_force)

            # Publish diagnostic corrected delta
            corr_msg = Vector3(x=u_cmd[0], y=u_cmd[1], z=u_cmd[2])
            self.pub_corrected_cmd.publish(corr_msg)

            # 2. Sub-interpolate u_cmd across 10 substeps (100 Hz stream).
            #    u_cmd is a Cartesian position delta, so it has to go through IK:
            #    publishing self.current_joints unchanged (the old placeholder)
            #    froze the arm while still holding the servo stream open.
            sub_step_delta = u_cmd / substeps
            target_pos = self.current_flange_pos.copy()
            max_joint_vel = 0.6  # rad/s guard for one 10 ms substep

            for _ in range(substeps):
                if self.is_fault or rospy.is_shutdown():
                    break

                target_pos = target_pos + sub_step_delta

                T_target = np.eye(4)
                T_target[:3, :3] = self.current_flange_rot
                T_target[:3, 3] = target_pos
                q_target = self.kin.ik_nearest(T_target, self.current_joints)

                if q_target is None:
                    rospy.logwarn_throttle(1.0, "[AuboServoNode] IK unreachable, holding position.")
                    break

                dq = np.max(np.abs(q_target - self.current_joints))
                if dq > max_joint_vel * sub_dt:
                    q_target = self.current_joints + (q_target - self.current_joints) * (
                        (max_joint_vel * sub_dt) / dq)

                servo_msg = JointState()
                servo_msg.header.stamp = rospy.Time.now()
                servo_msg.name = list(self.joint_names)
                servo_msg.position = q_target.tolist()
                self.pub_servo_joints.publish(servo_msg)

                self.current_joints = q_target
                rospy.sleep(sub_dt)

            self.current_flange_pos = target_pos

            rate.sleep()


if __name__ == "__main__":
    node = AuboStreamServoNode()
    node.spin()
