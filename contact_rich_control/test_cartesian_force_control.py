"""
Unit tests for Aubo-i10 Kinematics and 6-DoF Cartesian Admittance Controller.
"""

import math
import unittest
import numpy as np

from aubo_i10_kinematics import AuboI10Kinematics
from cartesian_admittance_controller import CartesianAdmittanceController, AdmittanceConfig


class TestAuboKinematics(unittest.TestCase):

    def setUp(self):
        self.kin = AuboI10Kinematics()

    def test_fk_and_ik_roundtrip(self):
        """Test FK followed by analytical IK recovers the original pose."""
        test_joints = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.2, -0.4, 0.8, -0.6, 0.5, -0.3]),
            np.array([-0.5, 0.3, -0.7, 0.4, -0.6, 0.2]),
            np.array([0.8, -0.8, 1.2, -0.9, 0.9, -0.5]),
        ]

        for q_orig in test_joints:
            T = self.kin.forward(q_orig)
            
            # Position must be reasonable
            self.assertTrue(np.all(np.isfinite(T)))
            self.assertAlmostEqual(T[3, 3], 1.0)
            
            # Determinant of rotation matrix must be 1.0
            det = np.linalg.det(T[:3, :3])
            self.assertAlmostEqual(det, 1.0, places=5)

            # Solve IK nearest
            q_ik = self.kin.ik_nearest(T, q_orig)
            self.assertIsNotNone(q_ik, f"IK failed for q={q_orig}")

            # FK on the IK solution must produce identical pose
            T_recovered = self.kin.forward(q_ik)
            pos_err = np.linalg.norm(T[:3, 3] - T_recovered[:3, 3])
            rot_err = np.linalg.norm(T[:3, :3] - T_recovered[:3, :3])

            self.assertLess(pos_err, 1e-4, f"Position error {pos_err} too large for q={q_orig}")
            self.assertLess(rot_err, 1e-4, f"Rotation error {rot_err} too large for q={q_orig}")

    def test_jacobian_finite_difference(self):
        """Verify Jacobian matches finite difference velocity."""
        q = np.array([0.1, -0.3, 0.5, -0.2, 0.4, -0.1])
        dq = np.array([0.01, -0.02, 0.015, -0.01, 0.02, -0.005])

        J = self.kin.jacobian(q)
        v_pred = J @ dq

        T0 = self.kin.forward(q)
        T1 = self.kin.forward(q + dq)
        v_actual_lin = (T1[:3, 3] - T0[:3, 3])

        lin_diff = np.linalg.norm(v_pred[:3] - v_actual_lin)
        self.assertLess(lin_diff, 1e-3)


class TestCartesianAdmittanceController(unittest.TestCase):

    def setUp(self):
        cfg = AdmittanceConfig(
            mass=np.array([5.0, 5.0, 5.0, 0.1, 0.1, 0.1]),
            stiffness=np.array([200.0, 200.0, 200.0, 10.0, 10.0, 10.0]),
            stiction_force=1.0,
            tool_mass=0.0,  # disable gravity comp for pure compliance testing
            max_force_threshold=120.0,
        )
        self.controller = CartesianAdmittanceController(config=cfg, dt=0.01)

    def test_free_space_transparency(self):
        """In free space (zero wrench), admittance output must track nominal pose without drift."""
        T_nom = np.eye(4)
        T_nom[:3, 3] = [0.5, 0.0, 0.3]
        wrench_zero = np.zeros(6)

        T_cmd = T_nom.copy()
        for _ in range(100):
            T_cmd, ok = self.controller.step(T_cmd, T_nom, wrench_zero)
            self.assertTrue(ok)

        # Output must be identical to nominal
        self.assertLess(np.linalg.norm(T_cmd[:3, 3] - T_nom[:3, 3]), 1e-6)

    def test_compliance_retreat_under_contact(self):
        """When an external force is applied (e.g. +20N in Z), controller should compliantly retreat."""
        T_nom = np.eye(4)
        T_nom[:3, 3] = [0.5, 0.0, 0.3]

        # Apply +20 N along Z
        wrench_contact = np.array([0.0, 0.0, 20.0, 0.0, 0.0, 0.0])
        T_curr = T_nom.copy()

        # Step 50 cycles (0.5s)
        for _ in range(50):
            T_curr, ok = self.controller.step(T_curr, T_nom, wrench_contact)
            self.assertTrue(ok)

        # Compliant deflection along Z:
        # Expected steady state disp: (20 - 1.0 stiction) / 200 stiffness = 0.095m, but clamped to max_linear_disp (0.05m)
        z_disp = T_curr[2, 3] - T_nom[2, 3]
        self.assertGreater(z_disp, 0.01, "Arm failed to retreat compliantly under 20N contact!")
        self.assertLessEqual(z_disp, 0.051, "Admittance displacement exceeded safety limit!")

    def test_safety_threshold_cutoff(self):
        """Exceeding 120N force limit must immediately flag safety_ok=False."""
        T_nom = np.eye(4)
        wrench_excessive = np.array([0.0, 0.0, 150.0, 0.0, 0.0, 0.0])

        _, ok = self.controller.step(T_nom, T_nom, wrench_excessive)
        self.assertFalse(ok, "Failed to flag safety violation under 150N load!")


if __name__ == "__main__":
    unittest.main()
