"""
Aubo-i10 Analytical Forward and Inverse Kinematics.
Strictly parameterized with official AUBO-i10 DH parameters from aubo_kinematics.

DH Parameters (AUBO-i10):
  a2 = 0.647 m   (upper arm length)
  a3 = 0.6005 m  (forearm length)
  d1 = 0.1632 m  (base height offset)
  d2 = 0.2013 m  (shoulder offset)
  d5 = 0.1025 m  (wrist2 offset)
  d6 = 0.094 m   (flange offset)

Joint limits: [-3.04 rad, +3.04 rad] for all 6 joints.
"""

import math
import numpy as np
from typing import Tuple, List, Optional


class AuboI10Kinematics:
    """Analytical Kinematics solver for AUBO-i10 robot arm."""

    # Official AUBO-i10 parameters
    A2 = 0.647
    A3 = 0.6005
    D1 = 0.1632
    D2 = 0.2013
    D5 = 0.1025
    D6 = 0.094

    JOINT_LIMIT_MIN = -3.04
    JOINT_LIMIT_MAX = 3.04
    ZERO_THRESH = 1e-6

    def __init__(self):
        pass

    # =========================================================================
    # Forward Kinematics
    # =========================================================================
    def forward(self, q: np.ndarray) -> np.ndarray:
        """Compute 4x4 homogeneous transformation matrix T from base to wrist3_Link.
        
        Args:
            q: 6-element joint angle vector in radians [q1, q2, q3, q4, q5, q6].
            
        Returns:
            T: 4x4 numpy array representing end-effector pose in base_link.
        """
        assert len(q) == 6, "q must have 6 joint angles"
        q1, q2, q3, q4, q5, q6 = q

        c1, s1 = math.cos(q1), math.sin(q1)
        c2, s2 = math.cos(q2), math.sin(q2)
        c4, s4 = math.cos(q4), math.sin(q4)
        c5, s5 = math.cos(q5), math.sin(q5)
        c6, s6 = math.cos(q6), math.sin(q6)

        c23 = math.cos(q2 - q3)
        s23 = math.sin(q2 - q3)
        c234 = math.cos(q2 - q3 + q4)
        s234 = math.sin(q2 - q3 + q4)
        c2345 = math.cos(q2 - q3 + q4 - q5)
        c2345p = math.cos(q2 - q3 + q4 + q5)

        a2, a3 = self.A2, self.A3
        d1, d2, d5, d6 = self.D1, self.D2, self.D5, self.D6

        T = np.zeros((4, 4), dtype=np.float64)

        T[0, 0] = -c6 * s1 * s5 + c1 * (c234 * c5 * c6 - s234 * s6)
        T[0, 1] = s1 * s5 * s6 - c1 * (c4 * c6 * s23 + c23 * c6 * s4 + c234 * c5 * s6)
        T[0, 2] = c5 * s1 + c1 * c234 * s5
        T[0, 3] = (d2 + c5 * d6) * s1 - c1 * (a2 * s2 + (a3 + c4 * d5) * s23 + c23 * d5 * s4 - c234 * d6 * s5)

        T[1, 0] = c234 * c5 * c6 * s1 + c1 * c6 * s5 - s1 * s234 * s6
        T[1, 1] = -c6 * s1 * s234 - (c234 * c5 * s1 + c1 * s5) * s6
        T[1, 2] = -c1 * c5 + c234 * s1 * s5
        T[1, 3] = -c1 * (d2 + c5 * d6) - s1 * (a2 * s2 + (a3 + c4 * d5) * s23 + c23 * d5 * s4 - c234 * d6 * s5)

        T[2, 0] = c5 * c6 * s234 + c234 * s6
        T[2, 1] = c234 * c6 - c5 * s234 * s6
        T[2, 2] = s234 * s5
        T[2, 3] = d1 + a2 * c2 + a3 * c23 + d5 * c234 + d6 * c2345 / 2.0 - d6 * c2345p / 2.0

        T[3, 3] = 1.0
        return T

    def forward_pos_quat(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Compute position [x, y, z] and quaternion [x, y, z, w]."""
        T = self.forward(q)
        pos = T[:3, 3].copy()
        quat = self.rot_to_quat(T[:3, :3])
        return pos, quat

    # =========================================================================
    # Inverse Kinematics
    # =========================================================================
    @staticmethod
    def _anti_sin_cos(s_a: float, c_a: float) -> float:
        eps = 1e-8
        if abs(s_a) < eps and abs(c_a) < eps:
            return 0.0
        return math.atan2(s_a, c_a)

    def inverse(self, T: np.ndarray) -> List[np.ndarray]:
        """Compute all valid analytical IK solutions (up to 8 solutions).
        
        Args:
            T: 4x4 target transformation matrix.
            
        Returns:
            List of 6-element joint angle numpy arrays.
        """
        nx, ox, ax, px = T[0, 0], T[0, 1], T[0, 2], T[0, 3]
        ny, oy, ay, py = T[1, 0], T[1, 1], T[1, 2], T[1, 3]
        nz, oz, az, pz = T[2, 0], T[2, 1], T[2, 2], T[2, 3]

        a2, a3 = self.A2, self.A3
        d1, d2, d5, d6 = self.D1, self.D2, self.D5, self.D6

        solutions: List[np.ndarray] = []

        # 1. Shoulder rotate joint (q1)
        A1 = d6 * ay - py
        B1 = d6 * ax - px
        R1 = A1 * A1 + B1 * B1 - d2 * d2

        if R1 < 0.0:
            return solutions

        R12 = math.sqrt(max(0.0, R1))
        q1_opts = [
            self._anti_sin_cos(A1, B1) - self._anti_sin_cos(d2, R12),
            self._anti_sin_cos(A1, B1) - self._anti_sin_cos(d2, -R12),
        ]
        q1_opts = [self._normalize_angle(val) for val in q1_opts]

        # 2. Wrist 2 joint (q5)
        for q1 in q1_opts:
            c1, s1 = math.cos(q1), math.sin(q1)
            b5 = -ay * c1 + ax * s1
            m5 = -ny * c1 + nx * s1
            n5 = -oy * c1 + ox * s1
            r5 = math.sqrt(m5 * m5 + n5 * n5)

            q5_opts = [
                self._anti_sin_cos(r5, b5),
                self._anti_sin_cos(-r5, b5),
            ]

            for q5 in q5_opts:
                s5 = math.sin(q5)
                a6 = -oy * c1 + ox * s1
                b6 = ny * c1 - nx * s1

                if abs(s5) < self.ZERO_THRESH:
                    # Singularity: q6 and q4 coupled
                    continue

                q6 = self._anti_sin_cos(a6 * s5, b6 * s5)
                c6, s6 = math.cos(q6), math.sin(q6)

                pp1 = c1 * (ax * d6 - px + d5 * ox * c6 + d5 * nx * s6) + s1 * (ay * d6 - py + d5 * oy * c6 + d5 * ny * s6)
                pp2 = -d1 - az * d6 + pz - d5 * oz * c6 - d5 * nz * s6
                b3 = (pp1 * pp1 + pp2 * pp2 - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)

                if (1.0 - b3 * b3) < -1e-6:
                    continue

                b3 = max(-1.0, min(1.0, b3))
                sin3 = math.sqrt(max(0.0, 1.0 - b3 * b3))

                q3_opts = [
                    self._anti_sin_cos(sin3, b3),
                    self._anti_sin_cos(-sin3, b3),
                ]

                for q3 in q3_opts:
                    c3, s3 = math.cos(q3), math.sin(q3)
                    a2_val = pp1 * (a2 + a3 * c3) + pp2 * (a3 * s3)
                    b2_val = pp2 * (a2 + a3 * c3) - pp1 * (a3 * s3)
                    q2 = self._anti_sin_cos(a2_val, b2_val)

                    c2, s2 = math.cos(q2), math.sin(q2)
                    a4 = -c1 * (ox * c6 + nx * s6) - s1 * (oy * c6 + ny * s6)
                    b4 = oz * c6 + nz * s6
                    a41 = pp1 - a2 * s2
                    b41 = pp2 - a2 * c2

                    q4 = self._normalize_angle(self._anti_sin_cos(a4, b4) - self._anti_sin_cos(a41, b41))

                    sol = np.array([q1, q2, q3, q4, q5, q6], dtype=np.float64)
                    if self._within_limits(sol):
                        solutions.append(sol)

        return solutions

    def ik_nearest(self, T: np.ndarray, q_curr: np.ndarray) -> Optional[np.ndarray]:
        """Find the analytical IK solution closest to current joint configuration.
        Falls back to Damped Least Squares (DLS) if near singular configuration.
        
        Args:
            T: 4x4 target transformation matrix.
            q_curr: 6-element current joint angles.
            
        Returns:
            Optimal 6-element joint angle vector, or None if unreachable.
        """
        solutions = self.inverse(T)
        if solutions:
            best_sol = None
            min_dist = float("inf")

            for sol in solutions:
                diff = sol - q_curr
                diff = np.arctan2(np.sin(diff), np.cos(diff))
                dist = np.linalg.norm(diff)
                if dist < min_dist:
                    min_dist = dist
                    best_sol = q_curr + diff

            return best_sol

        # Fallback: Damped Least Squares (DLS) Newton-Raphson numerical IK
        return self._dls_ik(T, q_curr)

    def _dls_ik(self, T_target: np.ndarray, q_init: np.ndarray, 
                max_iter: int = 40, tol: float = 1e-4, damping: float = 0.05) -> Optional[np.ndarray]:
        """Damped Least Squares IK solver for handling singularities and boundaries."""
        q = q_init.copy()
        p_target = T_target[:3, 3]
        R_target = T_target[:3, :3]

        for _ in range(max_iter):
            T_curr = self.forward(q)
            p_curr = T_curr[:3, 3]
            R_curr = T_curr[:3, :3]

            # Translation error
            dp = p_target - p_curr

            # Orientation error via small rotation matrix: R_err = R_target @ R_curr.T
            R_err = R_target @ R_curr.T
            dw = 0.5 * np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1]
            ])

            dx = np.concatenate([dp, dw])
            err_norm = np.linalg.norm(dx)
            if err_norm < tol:
                return q

            J = self.jacobian(q)
            # DLS step: dq = J^T (J J^T + lambda^2 I)^(-1) dx
            lambda_sq = damping * damping
            JJT_damped = J @ J.T + lambda_sq * np.eye(6)
            dq = J.T @ np.linalg.solve(JJT_damped, dx)

            # Limit max step
            dq_norm = np.linalg.norm(dq)
            if dq_norm > 0.1:
                dq = dq * (0.1 / dq_norm)

            q += dq

        return q if np.linalg.norm(dx) < 5e-3 else None

    # =========================================================================
    # Geometric Jacobian
    # =========================================================================
    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """Compute 6x6 geometric Jacobian J(q) mapping dq -> [v; w] in base_link."""
        delta = 1e-5
        J = np.zeros((6, 6), dtype=np.float64)
        T0 = self.forward(q)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]

        for i in range(6):
            q_plus = q.copy()
            q_plus[i] += delta
            T_plus = self.forward(q_plus)
            p_plus = T_plus[:3, 3]
            R_plus = T_plus[:3, :3]

            # Linear velocity column
            J[:3, i] = (p_plus - p0) / delta

            # Angular velocity column via small rotation vector: dR * R0^T
            dR = R_plus @ R0.T
            w_hat = (dR - dR.T) / (2.0 * delta)
            J[3, i] = w_hat[2, 1]
            J[4, i] = w_hat[0, 2]
            J[5, i] = w_hat[1, 0]

        return J

    # =========================================================================
    # Helpers
    # =========================================================================
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _within_limits(self, q: np.ndarray) -> bool:
        return bool(np.all(q >= self.JOINT_LIMIT_MIN - 1e-4) and np.all(q <= self.JOINT_LIMIT_MAX + 1e-4))

    @staticmethod
    def pose_to_matrix(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
        """Construct 4x4 matrix from position [x,y,z] and quaternion [x,y,z,w]."""
        x, y, z, w = quat
        # Normalize
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm > 1e-9:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        R = np.array([
            [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w),       2.0*(x*z + y*w)],
            [2.0*(x*y + z*w),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)],
            [2.0*(x*z - y*w),       2.0*(y*z + x*w),       1.0 - 2.0*(x*x + y*y)],
        ], dtype=np.float64)

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = pos
        return T

    @staticmethod
    def rot_to_quat(R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0.0:
            s = 0.5 / math.sqrt(tr + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        quat = np.array([x, y, z, w], dtype=np.float64)
        return quat / np.linalg.norm(quat)
