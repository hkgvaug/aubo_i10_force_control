#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aubo_cartesian_force_servo_node.py — 6-DoF Cartesian Admittance & CAN Stream Servo Node.

Architecture:
  1. High-Level Target: Subscribes to /bim/target_pose (from LLM/BIM planning).
  2. Force Sensor Stream: Subscribes to /ft_sensor/wrench (250 Hz from KWR75).
  3. Safety Supervisor: Subscribes to /ft_safety/state (interlocked).
  4. Real-time Admittance: Runs 100 Hz closed-loop SE(3) admittance and gravity compensation.
  5. Kinematics & Dispatch: Solves AUBO-i10 analytical IK and streams joint targets directly
     to aubo_driver (bypassing MoveIt's 600ms-1.2s trajectory buffer).
"""

import os
import sys
import time
import math
import json
import threading
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectoryPoint

# ── Import Kinematics and Admittance Controller ───────────────────────────────
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

PKG_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../.."))
CTRL_DIR = os.path.join(PKG_DIR, "contact_rich_control")
if CTRL_DIR not in sys.path:
    sys.path.insert(0, CTRL_DIR)

from aubo_i10_kinematics import AuboI10Kinematics
from cartesian_admittance_controller import CartesianAdmittanceController, AdmittanceConfig


class AuboCartesianForceServoNode:
    """Production 100 Hz Cartesian Admittance and Streaming Servo Node."""

    def __init__(self):
        rospy.init_node("aubo_cartesian_force_servo_node", anonymous=False)

        # ── Node Parameters ──────────────────────────────────────────────────
        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", 100.0))
        self.dt = 1.0 / self.loop_rate_hz

        self.mock_hardware = bool(rospy.get_param("~mock_hardware", False))
        self.robot_ip = str(rospy.get_param("~robot_ip", "192.168.100.11"))

        # Safety & Velocity Limits
        self.max_lin_vel = float(rospy.get_param("~max_linear_velocity", 0.08))       # 80 mm/s
        self.max_ang_vel = float(rospy.get_param("~max_angular_velocity", 0.35))      # ~20 deg/s
        self.max_step_disp = float(rospy.get_param("~max_step_displacement", 0.002))  # 2 mm per step
        self.max_joint_jump = float(rospy.get_param("~max_joint_jump_rad", 0.15))     # IK branch guard
        self.max_force_cutoff = float(rospy.get_param("~max_force_cutoff", 100.0))    # 100 N
        self.max_torque_cutoff = float(rospy.get_param("~max_torque_cutoff", 5.0))    # 5 Nm

        # Target Force parameters (e.g. for surface wiping or pressing)
        target_f_z = float(rospy.get_param("~target_force_z", 0.0))
        self.wrench_target = np.array([0.0, 0.0, target_f_z, 0.0, 0.0, 0.0], dtype=np.float64)

        # ── Initialize Admittance & Kinematics ────────────────────────────────
        adm_cfg = AdmittanceConfig(
            mass=np.array(rospy.get_param("~admittance_mass", [5.0, 5.0, 5.0, 0.1, 0.1, 0.1]), dtype=np.float64),
            stiffness=np.array(rospy.get_param("~admittance_stiffness", [250.0, 250.0, 250.0, 15.0, 15.0, 15.0]), dtype=np.float64),
            damping_ratio=np.array(rospy.get_param("~admittance_damping_ratio", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), dtype=np.float64),
            stiction_force=float(rospy.get_param("~stiction_force", 1.5)),
            stiction_torque=float(rospy.get_param("~stiction_torque", 0.08)),
            tool_mass=float(rospy.get_param("~tool_mass", 0.95)),
            tool_cog=np.array(rospy.get_param("~tool_cog", [0.0, 0.0, 0.045]), dtype=np.float64),
            max_force_threshold=self.max_force_cutoff,
            max_torque_threshold=self.max_torque_cutoff,
        )

        self.kin = AuboI10Kinematics()
        self.controller = CartesianAdmittanceController(config=adm_cfg, dt=self.dt)

        # /ft_sensor/wrench is already gravity-compensated once /ft_sensor/tare has
        # been called, so running the analytic gravity compensation on top of it
        # double-counts the tool weight and makes the arm drift.  Enable it only
        # when subscribing to the untared raw topic.
        self.enable_gravity_comp = bool(rospy.get_param("~enable_gravity_comp", False))
        # 0 disables the check (not recommended: it removes the interlock).
        self.safety_state_age_limit = float(rospy.get_param("~safety_state_age_limit", 1.0))

        # ── State Variables ──────────────────────────────────────────────────
        self.lock = threading.Lock()
        self.current_joints = np.array([0.0, -0.3, 1.0, -0.7, 1.5708, 0.0], dtype=np.float64)
        self.joint_states_received = False

        self.measured_wrench = np.zeros(6, dtype=np.float64)  # in sensor frame
        self.last_ft_time = rospy.Time.now()

        self.safety_state = "NORMAL"
        self.last_safety_time = rospy.Time(0)
        self.safety_state_received = False
        self.is_fault = False

        # Current & Nominal Poses (4x4 matrices)
        self.T_curr = self.kin.forward(self.current_joints)
        self.T_nom = self.T_curr.copy()
        self.T_goal = self.T_curr.copy()
        self.goal_active = False

        # ── Direct C SDK Integration (Optional) ──────────────────────────────
        self.c_sdk_active = False
        self.rshd = None
        if not self.mock_hardware:
            self._try_init_c_sdk()

        # ── ROS Publishers ───────────────────────────────────────────────────
        # The driver drains ~7.5 points/s.  Publishing the 100 Hz admittance
        # stream to moveItController_cmd (the buffered trajectory queue) builds an
        # unbounded backlog, so it is off by default: the dedicated servo topic is
        # latest-wins and bypasses the queue.  Enable only if servo_joint_cmd is
        # not consumed by your driver.
        self.publish_moveit_points = bool(rospy.get_param("~publish_moveit_points", False))
        self.pub_servo_joint = rospy.Publisher("/aubo_driver/servo_joint_cmd", JointState, queue_size=1)
        self.pub_moveit_point = None
        if self.publish_moveit_points:
            self.pub_moveit_point = rospy.Publisher("moveItController_cmd", JointTrajectoryPoint, queue_size=5)
        self.pub_curr_pose = rospy.Publisher("/cartesian_force_servo/current_pose", PoseStamped, queue_size=1)
        self.pub_cmd_pose = rospy.Publisher("/cartesian_force_servo/cmd_pose", PoseStamped, queue_size=1)
        self.pub_base_wrench = rospy.Publisher("/cartesian_force_servo/wrench_base", WrenchStamped, queue_size=1)
        self.pub_status = rospy.Publisher("/cartesian_force_servo/status", String, queue_size=1)

        # ── ROS Subscribers ──────────────────────────────────────────────────
        rospy.Subscriber("/bim/target_pose", PoseStamped, self._on_target_pose, queue_size=10)
        rospy.Subscriber("/ft_sensor/wrench", WrenchStamped, self._on_ft_sensor, queue_size=1)
        rospy.Subscriber("/ft_safety/state", String, self._on_safety_state, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self._on_joint_states, queue_size=1)

        rospy.loginfo(f"[CartesianForceServo] Initialized at {self.loop_rate_hz}Hz (Mock: {self.mock_hardware})")

    # =========================================================================
    # Callbacks
    # =========================================================================
    def _on_target_pose(self, msg: PoseStamped):
        """Receive nominal target pose from LLM/BIM planner."""
        with self.lock:
            pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
            quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, 
                             msg.pose.orientation.z, msg.pose.orientation.w], dtype=np.float64)

            # Check if valid orientation provided, otherwise keep current orientation
            if np.linalg.norm(quat) < 0.1:
                quat = AuboI10Kinematics.rot_to_quat(self.T_nom[:3, :3])

            self.T_goal = AuboI10Kinematics.pose_to_matrix(pos, quat)
            self.goal_active = True
            rospy.loginfo(f"[CartesianForceServo] New Target Pose: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

    def _on_ft_sensor(self, msg: WrenchStamped):
        """Receive tare-corrected 6-axis wrench from KWR75 (250 Hz)."""
        with self.lock:
            self.last_ft_time = rospy.Time.now()
            self.measured_wrench = np.array([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ], dtype=np.float64)

    def _on_safety_state(self, msg: String):
        """Safety interlock with ft_safety_monitor."""
        self.last_safety_time = rospy.Time.now()
        self.safety_state = msg.data.upper()
        self.safety_state_received = True
        if "FAULT" in self.safety_state:
            if not self.is_fault:
                rospy.logerr(f"[CartesianForceServo] FAULT received from ft_safety_monitor: {msg.data}! Halting.")
            self.is_fault = True
        else:
            self.is_fault = False

    def _on_joint_states(self, msg: JointState):
        """Track current actual robot joint positions."""
        if len(msg.position) >= 6:
            with self.lock:
                self.current_joints = np.array(msg.position[:6], dtype=np.float64)
                self.T_curr = self.kin.forward(self.current_joints)
                if not self.joint_states_received:
                    self.T_nom = self.T_curr.copy()
                    self.T_goal = self.T_curr.copy()
                    self.joint_states_received = True

    # =========================================================================
    # C SDK Hardware Init (Optional Direct Link)
    # =========================================================================
    def _try_init_c_sdk(self):
        """Optionally connect to libauborobotcontroller.so directly."""
        try:
            import ctypes
            lib_path = "/usr/lib/libauborobotcontroller.so"
            if not os.path.exists(lib_path):
                # Search workspace
                ws_lib = os.path.join(PKG_DIR, "aubo_ws/devel/lib/libauborobotcontroller.so")
                if os.path.exists(ws_lib):
                    lib_path = ws_lib
            if os.path.exists(lib_path):
                # We can load it if direct C calling is enabled
                rospy.loginfo(f"[CartesianForceServo] Aubo C SDK library detected at {lib_path}")
        except Exception as e:
            rospy.logwarn(f"[CartesianForceServo] C SDK optional link: {e}")

    # =========================================================================
    # Main Real-Time 100 Hz Loop
    # =========================================================================
    def spin(self):
        rate = rospy.Rate(self.loop_rate_hz)
        rospy.loginfo("[CartesianForceServo] Control Loop Started.")

        while not rospy.is_shutdown():
            loop_start = time.monotonic()

            with self.lock:
                curr_joints = self.current_joints.copy()
                curr_wrench = self.measured_wrench.copy()
                T_current = self.T_curr.copy()
                T_nominal = self.T_nom.copy()
                T_target_goal = self.T_goal.copy()

            # 1. Check Safety Interlocks
            if self.is_fault:
                rate.sleep()
                continue

            # Fail closed: without a fresh /ft_safety/state the interlock is
            # absent, so no motion is allowed.  The previous version treated a
            # missing monitor as NORMAL, i.e. it silently disabled the interlock.
            safety_age = (rospy.Time.now() - self.last_safety_time).to_sec()
            if self.safety_state_age_limit > 0.0 and safety_age > self.safety_state_age_limit:
                rospy.logwarn_throttle(
                    2.0,
                    f"[CartesianForceServo] No /ft_safety/state for {safety_age:.1f}s "
                    "(is ft_safety_monitor running?); holding position.")
                rate.sleep()
                continue

            # Never command before the real joint state is known: the default
            # seed pose would otherwise be streamed to the robot as a target and
            # make it lurch towards an arbitrary configuration.
            if not self.joint_states_received and not self.mock_hardware:
                rospy.logwarn_throttle(2.0, "[CartesianForceServo] Waiting for /joint_states from aubo_driver; holding.")
                rate.sleep()
                continue

            # F/T Sensor Watchdog (0.5s timeout)
            ft_age = (rospy.Time.now() - self.last_ft_time).to_sec()
            if ft_age > 0.5:
                rospy.logwarn_throttle(2.0, f"[CartesianForceServo] FT sensor timeout (age={ft_age:.2f}s)! Pausing.")
                rate.sleep()
                continue

            # 2. Interpolate Nominal Trajectory towards Goal (Velocity-Limited)
            if self.goal_active:
                p_nom = T_nominal[:3, 3]
                p_goal = T_target_goal[:3, 3]
                dp = p_goal - p_nom
                dist = np.linalg.norm(dp)

                max_step = self.max_lin_vel * self.dt
                if dist > max_step:
                    p_nom_next = p_nom + (dp / dist) * max_step
                else:
                    p_nom_next = p_goal
                    self.goal_active = False

                T_nominal[:3, 3] = p_nom_next
                # Slerp / direct orientation target
                T_nominal[:3, :3] = T_target_goal[:3, :3]
                self.T_nom = T_nominal.copy()

            # 3. Cartesian Admittance Step
            T_cmd, safety_ok = self.controller.step(
                T_curr=T_current,
                T_nom=T_nominal,
                wrench_sensor=curr_wrench,
                wrench_target=self.wrench_target,
                enable_gravity_comp=self.enable_gravity_comp
            )

            if not safety_ok:
                rospy.logerr_throttle(1.0, "[CartesianForceServo] Force safety limit exceeded! Freezing motion.")
                rate.sleep()
                continue

            # 4. Step Displacement Clamping (Single-cycle displacement guard)
            p_step = T_cmd[:3, 3] - T_current[:3, 3]
            step_norm = np.linalg.norm(p_step)
            if step_norm > self.max_step_disp:
                T_cmd[:3, 3] = T_current[:3, 3] + (p_step / step_norm) * self.max_step_disp

            # 5. Solve Analytical Inverse Kinematics
            q_target = self.kin.ik_nearest(T_cmd, curr_joints)
            if q_target is None:
                rospy.logwarn_throttle(1.0, "[CartesianForceServo] IK unreachable or singular! Holding position.")
                rate.sleep()
                continue

            # Reject configuration flips instead of smoothing them: near
            # singularities ik_nearest can return a different branch, and the
            # velocity clip below would then drive the arm through the flip.
            if np.max(np.abs(q_target - curr_joints)) > self.max_joint_jump:
                rospy.logwarn_throttle(
                    1.0,
                    "[CartesianForceServo] IK branch jump %.2f rad rejected; holding position."
                    % np.max(np.abs(q_target - curr_joints)))
                rate.sleep()
                continue

            # 6. Joint Velocity Limit Check
            dq = (q_target - curr_joints) / self.dt
            max_dq = np.max(np.abs(dq))
            if max_dq > 2.5:  # rad/s safety limit
                scale = 2.5 / max_dq
                q_target = curr_joints + (q_target - curr_joints) * scale

            # 7. Dispatch Commands to Robot
            if self.mock_hardware:
                # Update simulated joints
                with self.lock:
                    self.current_joints = q_target.copy()
                    self.T_curr = self.kin.forward(self.current_joints)
            else:
                # Send to aubo_driver via ROS topics
                joint_msg = JointState()
                joint_msg.header.stamp = rospy.Time.now()
                joint_msg.name = ["shoulder_joint", "upperArm_joint", "foreArm_joint", 
                                  "wrist1_joint", "wrist2_joint", "wrist3_joint"]
                joint_msg.position = q_target.tolist()
                self.pub_servo_joint.publish(joint_msg)

                # Optional legacy path; JointTrajectoryPoint needs accelerations
                # as well, otherwise moveItPosCallback reads past the vector end.
                if self.pub_moveit_point is not None:
                    traj_pt = JointTrajectoryPoint()
                    traj_pt.positions = q_target.tolist()
                    traj_pt.velocities = ((q_target - curr_joints) / self.dt).tolist()
                    traj_pt.accelerations = [0.0] * len(q_target)
                    traj_pt.time_from_start = rospy.Duration.from_sec(self.dt)
                    self.pub_moveit_point.publish(traj_pt)

            # 8. Publish Telemetry Diagnostics (10 Hz throttled)
            if rospy.Time.now().to_sec() % 0.1 < self.dt:
                self._publish_diagnostics(T_current, T_cmd)

            rate.sleep()

    def _publish_diagnostics(self, T_curr: np.ndarray, T_cmd: np.ndarray):
        """Publish real-time telemetry."""
        now = rospy.Time.now()

        # Current pose
        curr_msg = PoseStamped()
        curr_msg.header.stamp = now
        curr_msg.header.frame_id = "base_link"
        curr_msg.pose.position = Point(x=T_curr[0, 3], y=T_curr[1, 3], z=T_curr[2, 3])
        q = AuboI10Kinematics.rot_to_quat(T_curr[:3, :3])
        curr_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.pub_curr_pose.publish(curr_msg)

        # Commanded pose
        cmd_msg = PoseStamped()
        cmd_msg.header.stamp = now
        cmd_msg.header.frame_id = "base_link"
        cmd_msg.pose.position = Point(x=T_cmd[0, 3], y=T_cmd[1, 3], z=T_cmd[2, 3])
        qc = AuboI10Kinematics.rot_to_quat(T_cmd[:3, :3])
        cmd_msg.pose.orientation = Quaternion(x=qc[0], y=qc[1], z=qc[2], w=qc[3])
        self.pub_cmd_pose.publish(cmd_msg)

        # Base frame wrench
        w_base = self.controller.last_contact_wrench
        wb_msg = WrenchStamped()
        wb_msg.header.stamp = now
        wb_msg.header.frame_id = "base_link"
        wb_msg.wrench.force = Vector3(x=w_base[0], y=w_base[1], z=w_base[2])
        wb_msg.wrench.torque = Vector3(x=w_base[3], y=w_base[4], z=w_base[5])
        self.pub_base_wrench.publish(wb_msg)

        # Status JSON
        diag = {
            "status": "NORMAL" if not self.is_fault else "FAULT",
            "contact_force_norm": float(np.linalg.norm(w_base[:3])),
            "contact_torque_norm": float(np.linalg.norm(w_base[3:])),
            "admittance_disp_mm": float(np.linalg.norm(self.controller.x_adm[:3]) * 1000.0),
        }
        self.pub_status.publish(String(data=json.dumps(diag)))


if __name__ == "__main__":
    node = AuboCartesianForceServoNode()
    node.spin()
