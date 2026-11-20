"""
6-DoF Cartesian Admittance & Hybrid Force-Velocity Controller.
Based on the SE(3) Lie Group Admittance formulation from force_control-mainline
and ros2_controllers / cartesian_controllers.

Key equations:
  1. Adjoint Transform:
     F_base = R_base_sensor * F_sensor
     tau_base = R_base_sensor * tau_sensor + p_sensor_base x F_base
  2. Tool Gravity & CoG Compensation:
     F_gravity_base = [0, 0, -m_tool * g]
     tau_gravity_base = (R_base_tool * r_cog) x F_gravity_base
     W_contact = W_base - W_gravity
  3. Admittance Law:
     M * x_ddot + D * x_dot + K * (x - x_nom) = W_contact - W_target
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional


@dataclass
class AdmittanceConfig:
    """Configuration parameters for 6-DoF Admittance Controller."""
    # Virtual Inertia M (diag: 3 translation [kg], 3 rotation [kg*m^2])
    mass: np.ndarray = field(default_factory=lambda: np.array([5.0, 5.0, 5.0, 0.1, 0.1, 0.1], dtype=np.float64))
    
    # Virtual Stiffness K (diag: 3 translation [N/m], 3 rotation [Nm/rad])
    stiffness: np.ndarray = field(default_factory=lambda: np.array([300.0, 300.0, 300.0, 15.0, 15.0, 15.0], dtype=np.float64))
    
    # Critical Damping Ratio zeta (1.0 = critically damped, avoids oscillations)
    damping_ratio: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float64))

    # Deadzone thresholds (stiction deadband to reject sensor measurement noise)
    stiction_force: float = 1.5   # Newtons
    stiction_torque: float = 0.08  # Newton-meters

    # Safety clamps
    max_linear_velocity: float = 0.15     # m/s (150 mm/s)
    max_angular_velocity: float = 0.50    # rad/s (~28 deg/s)
    max_linear_disp: float = 0.050        # max admittance displacement (50 mm)
    max_angular_disp: float = 0.20        # max admittance angular displacement (~11 deg)
    max_force_threshold: float = 120.0    # Emergency stop force limit (N)
    max_torque_threshold: float = 6.0     # Emergency stop torque limit (Nm)

    # Tool properties for gravity & CoG compensation
    tool_mass: float = 0.95               # kg (gripper + sensor adapter)
    tool_cog: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.045], dtype=np.float64))  # meters in tool frame
    gravity_accel: float = 9.81           # m/s^2

    # Hybrid Force-Velocity axis mask: 1 = Force-controlled, 0 = Position-controlled
    # e.g. [0, 0, 1, 0, 0, 0] means Z axis tracks force, others track position
    force_axis_mask: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 0, 0, 0], dtype=np.int32))


class CartesianAdmittanceController:
    """6-DoF Cartesian Admittance Controller."""

    def __init__(self, config: Optional[AdmittanceConfig] = None, dt: float = 0.01):
        self.cfg = config or AdmittanceConfig()
        self.dt = dt

        # Compute Damping matrix D = 2 * zeta * sqrt(M * K)
        self.damping = 2.0 * self.cfg.damping_ratio * np.sqrt(self.cfg.mass * self.cfg.stiffness)

        # Admittance state in base frame: [dx, dy, dz, drx, dry, drz]
        self.x_adm = np.zeros(6, dtype=np.float64)
        self.v_adm = np.zeros(6, dtype=np.float64)
        self.a_adm = np.zeros(6, dtype=np.float64)

        # Diagnostic state
        self.last_contact_wrench = np.zeros(6, dtype=np.float64)
        self.last_gravity_wrench = np.zeros(6, dtype=np.float64)

    def reset(self):
        """Reset admittance state to zero."""
        self.x_adm.fill(0.0)
        self.v_adm.fill(0.0)
        self.a_adm.fill(0.0)

    # =========================================================================
    # Wrench Transformations & Gravity Compensation
    # =========================================================================
    def transform_wrench_to_base(self, wrench_sensor: np.ndarray, R_base_sensor: np.ndarray,
                                 p_sensor_base: Optional[np.ndarray] = None) -> np.ndarray:
        """Transform 6D wrench from sensor coordinate frame to base_link frame.
        
        Args:
            wrench_sensor: 6-element array [Fx, Fy, Fz, Tx, Ty, Tz] in sensor frame.
            R_base_sensor: 3x3 rotation matrix from sensor to base frame.
            p_sensor_base: 3-element position of sensor origin relative to base, optional.
            
        Returns:
            wrench_base: 6-element array [Fx, Fy, Fz, Tx, Ty, Tz] in base frame.
        """
        f_s = wrench_sensor[:3]
        tau_s = wrench_sensor[3:]

        # Rotate forces and torques
        f_b = R_base_sensor @ f_s
        tau_b = R_base_sensor @ tau_s

        if p_sensor_base is not None:
            # Add cross product torque: tau_b += p x f_b
            tau_b += np.cross(p_sensor_base, f_b)

        wrench_base = np.zeros(6, dtype=np.float64)
        wrench_base[:3] = f_b
        wrench_base[3:] = tau_b
        return wrench_base

    def compute_gravity_compensation(self, R_base_tool: np.ndarray) -> np.ndarray:
        """Compute wrench generated by tool payload mass & CoG under gravity in base frame."""
        f_grav = np.array([0.0, 0.0, -self.cfg.tool_mass * self.cfg.gravity_accel], dtype=np.float64)
        r_cog_base = R_base_tool @ self.cfg.tool_cog
        tau_grav = np.cross(r_cog_base, f_grav)

        w_grav = np.zeros(6, dtype=np.float64)
        w_grav[:3] = f_grav
        w_grav[3:] = tau_grav
        return w_grav

    # =========================================================================
    # Admittance Step
    # =========================================================================
    def step(self, 
             T_curr: np.ndarray, 
             T_nom: np.ndarray, 
             wrench_sensor: np.ndarray, 
             wrench_target: Optional[np.ndarray] = None,
             enable_gravity_comp: bool = True) -> Tuple[np.ndarray, bool]:
        """Execute one admittance cycle at rate 1/dt.
        
        Args:
            T_curr: 4x4 current measured flange transformation in base_link.
            T_nom: 4x4 nominal goal transformation from trajectory planner.
            wrench_sensor: 6-element array [Fx, Fy, Fz, Tx, Ty, Tz] from KWR75 sensor.
            wrench_target: 6-element desired contact wrench (default 0).
            enable_gravity_comp: whether to subtract tool weight & CoG moment.
            
        Returns:
            T_cmd: 4x4 commanded transformation to send to robot.
            safety_ok: False if wrench exceeded max threshold, True otherwise.
        """
        R_b = T_curr[:3, :3]
        p_b = T_curr[:3, 3]

        if wrench_target is None:
            wrench_target = np.zeros(6, dtype=np.float64)

        # 1. Transform raw wrench to base frame
        w_base = self.transform_wrench_to_base(wrench_sensor, R_b)

        # 2. Gravity compensation
        if enable_gravity_comp and self.cfg.tool_mass > 0.0:
            w_grav = self.compute_gravity_compensation(R_b)
            self.last_gravity_wrench = w_grav.copy()
            w_contact = w_base - w_grav
        else:
            w_contact = w_base.copy()

        self.last_contact_wrench = w_contact.copy()

        # 3. Safety threshold check
        force_norm = np.linalg.norm(w_contact[:3])
        torque_norm = np.linalg.norm(w_contact[3:])
        if force_norm > self.cfg.max_force_threshold or torque_norm > self.cfg.max_torque_threshold:
            return T_curr.copy(), False

        # 4. Deadzone filter (reject measurement noise)
        w_effective = w_contact - wrench_target
        for i in range(3):
            if abs(w_effective[i]) < self.cfg.stiction_force:
                w_effective[i] = 0.0
            else:
                w_effective[i] -= math.copysign(self.cfg.stiction_force, w_effective[i])
        for i in range(3, 6):
            if abs(w_effective[i]) < self.cfg.stiction_torque:
                w_effective[i] = 0.0
            else:
                w_effective[i] -= math.copysign(self.cfg.stiction_torque, w_effective[i])

        # 5. Admittance differential equation:
        # M * a_adm + D * v_adm + K * x_adm = w_effective
        # => a_adm = (w_effective - D * v_adm - K * x_adm) / M
        a_new = (w_effective - self.damping * self.v_adm - self.cfg.stiffness * self.x_adm) / self.cfg.mass

        # Euler integration
        self.v_adm += a_new * self.dt

        # Velocity clamping
        lin_speed = np.linalg.norm(self.v_adm[:3])
        if lin_speed > self.cfg.max_linear_velocity:
            self.v_adm[:3] *= (self.cfg.max_linear_velocity / lin_speed)

        ang_speed = np.linalg.norm(self.v_adm[3:])
        if ang_speed > self.cfg.max_angular_velocity:
            self.v_adm[3:] *= (self.cfg.max_angular_velocity / ang_speed)

        self.x_adm += self.v_adm * self.dt

        # Displacement clamping
        lin_disp = np.linalg.norm(self.x_adm[:3])
        if lin_disp > self.cfg.max_linear_disp:
            self.x_adm[:3] *= (self.cfg.max_linear_disp / lin_disp)

        ang_disp = np.linalg.norm(self.x_adm[3:])
        if ang_disp > self.cfg.max_angular_disp:
            self.x_adm[3:] *= (self.cfg.max_angular_disp / ang_disp)

        # 6. Apply Admittance displacement to Nominal Pose on SE(3)
        # Translation addition:
        p_cmd = T_nom[:3, 3] + self.x_adm[:3]

        # Orientation perturbation via Rodrigues rotation vector:
        rot_vec = self.x_adm[3:]
        theta = np.linalg.norm(rot_vec)
        if theta > 1e-6:
            axis = rot_vec / theta
            # Rodrigues formula
            K_skew = np.array([
                [0.0, -axis[2], axis[1]],
                [axis[2], 0.0, -axis[0]],
                [-axis[1], axis[0], 0.0]
            ], dtype=np.float64)
            R_delta = np.eye(3) + math.sin(theta) * K_skew + (1.0 - math.cos(theta)) * (K_skew @ K_skew)
            R_cmd = R_delta @ T_nom[:3, :3]
        else:
            R_cmd = T_nom[:3, :3].copy()

        T_cmd = np.eye(4, dtype=np.float64)
        T_cmd[:3, :3] = R_cmd
        T_cmd[:3, 3] = p_cmd

        return T_cmd, True
