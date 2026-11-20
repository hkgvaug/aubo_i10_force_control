"""
L1-Inspired Adaptive Controller for Contact-Rich Robotic Manipulation.
Implementation strictly matching the formulation in:
"Contact-Rich Robotic Manipulation in Construction via Zero-Shot Learning:
A Diffusion Policy-Guided Adaptive Control" (Ibrahimov, Mozaffari, Adel, Princeton University)

Formulas from Section 3.4 (Eqs. 3 - 14):
  - State Predictor (Eq. 4)
  - Prediction Error (Eq. 5)
  - Adaptation Law with tanh force gating & leakage decay (Eqs. 6 - 8)
  - 1st-order Low-Pass Filter (Eqs. 9 - 10)
  - Force-based scaling factor beta (Eqs. 11 - 12)
  - Safety box clipping PU (Eqs. 13 - 14)
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class L1ControllerConfig:
    """Hyperparameters for L1-inspired adaptive controller.
    
    Default parameters are taken from Table 2 of the paper for Timber Joint:
      - apred: -62.8
      - gamma_gain (Γ): 5.0 (timber) or 80.0 (pipe)
      - lambda_decay (λ): 0.98 (timber) or 0.99 (pipe)
      - kd: 0.005 N^-1
      - F0: 30.0 N
      - sigma_max: 0.001 m (1 mm)
      - delta_max: 0.010 m (10 mm)
      - cutoff_freq_fc: 10.0 Hz
      - predictor_drift_bound: 0.050 m (50 mm)
    """
    apred: float = -62.8
    gamma_gain: float = 5.0
    lambda_decay: float = 0.98
    kd: float = 0.005
    F0: float = 30.0
    sigma_max: float = 0.001          # 1 mm in meters
    delta_max: float = 0.010          # 10 mm in meters
    cutoff_freq_fc: float = 10.0      # Hz
    predictor_drift_bound: float = 0.050  # 50 mm in meters


class L1AdaptiveController:
    """L1-inspired adaptive controller for 3D Cartesian position/action corrections."""

    def __init__(self, config: L1ControllerConfig = None, dt: float = 0.1):
        """
        Args:
            config: L1ControllerConfig instance with controller gains.
            dt: Discrete sampling period in seconds (default 0.1s for 10 Hz loop).
        """
        self.cfg = config if config is not None else L1ControllerConfig()
        self.dt = dt

        # State predictor state p_hat (3D position)
        self.p_hat = np.zeros(3, dtype=np.float64)
        # Raw disturbance estimate sigma_hat
        self.sigma_hat = np.zeros(3, dtype=np.float64)
        # Filtered disturbance estimate sigma_filtered
        self.sigma_filtered = np.zeros(3, dtype=np.float64)
        
        # Previous commanded action u_cmd
        self.u_cmd_prev = np.zeros(3, dtype=np.float64)

        # Flag indicating whether predictor has been initialized
        self.is_initialized = False

        # Filter coefficient gamma (Eq. 10): gamma = (omega_c * dt) / (1 + omega_c * dt)
        omega_c = 2.0 * np.pi * self.cfg.cutoff_freq_fc
        self.filter_gamma = (omega_c * self.dt) / (1.0 + omega_c * self.dt)

    def reset(self, p_init: np.ndarray = None):
        """Reset internal states at the beginning of each rollout."""
        if p_init is not None:
            self.p_hat = np.asarray(p_init, dtype=np.float64).copy()
            self.is_initialized = True
        else:
            self.p_hat.fill(0.0)
            self.is_initialized = False

        self.sigma_hat.fill(0.0)
        self.sigma_filtered.fill(0.0)
        self.u_cmd_prev.fill(0.0)

    def step(self, p_curr: np.ndarray, u_pol: np.ndarray, F_meas: np.ndarray) -> np.ndarray:
        """Execute one discrete control cycle (Euler integration).
        
        Args:
            p_curr: Measured robot flange position (3D, meters)
            u_pol: Nominal action predicted by Diffusion Policy (3D delta position or velocity * dt)
            F_meas: Measured contact force at flange (3D force vector, Newtons)
            
        Returns:
            u_cmd: Corrected command action to send to robot (3D, meters)
        """
        p_curr = np.asarray(p_curr, dtype=np.float64)
        u_pol = np.asarray(u_pol, dtype=np.float64)
        F_meas = np.asarray(F_meas, dtype=np.float64)

        # Initialize predictor position if first call
        if not self.is_initialized:
            self.p_hat = p_curr.copy()
            self.is_initialized = True

        # Compute force magnitude ||F(t)||
        f_norm = float(np.linalg.norm(F_meas))

        # --- 1. State Predictor Update (Eq. 4 & Sanity Check) ---
        # e(t) = p_hat(t) - p(t)
        e = self.p_hat - p_curr

        # Predictor drift sanity check (50 mm): reset if exceeds bound
        if np.linalg.norm(e) > self.cfg.predictor_drift_bound:
            self.p_hat = p_curr.copy()
            e = np.zeros(3, dtype=np.float64)

        # Predictor derivative: dot_p_hat = Apred * e + Bpred * u_cmd_prev + sigma_filtered
        # Note: Apred = apred * I, Bpred = I
        dot_p_hat = (self.cfg.apred * e) + self.u_cmd_prev + self.sigma_filtered
        # Forward Euler update for p_hat
        self.p_hat += dot_p_hat * self.dt

        # Recompute prediction error after state evolution
        e = self.p_hat - p_curr

        # --- 2. Adaptation Law (Eqs. 6 - 8) ---
        # Force activation alpha(t) = tanh(||F(t)|| / F0)
        alpha = np.tanh(f_norm / max(self.cfg.F0, 1e-6))

        # dot_sigma_hat = Gamma * e * alpha - lambda * sigma_hat
        dot_sigma_hat = (self.cfg.gamma_gain * e * alpha) - (self.cfg.lambda_decay * self.sigma_hat)
        self.sigma_hat += dot_sigma_hat * self.dt

        # Disturbance Projection (Eq. 8)
        sigma_norm = float(np.linalg.norm(self.sigma_hat))
        if sigma_norm <= self.cfg.sigma_max:
            sigma_proj = self.sigma_hat.copy()
        else:
            sigma_proj = self.sigma_hat * (self.cfg.sigma_max / max(sigma_norm, 1e-12))

        # --- 3. Discrete Low-Pass Filter (Eq. 9) ---
        self.sigma_filtered = (self.filter_gamma * sigma_proj) + ((1.0 - self.filter_gamma) * self.sigma_filtered)

        # --- 4. Control Law (Eqs. 11 - 14) ---
        # Force damping factor beta(t) = 1 / (1 + kd * ||F(t)||)
        beta = 1.0 / (1.0 + self.cfg.kd * f_norm)

        # Corrected action u_corr(t) = u_pol(t) * beta(t) + sigma_filtered(t)
        u_corr = (u_pol * beta) + self.sigma_filtered

        # Safety projection PU (Eqs. 13 - 14) onto [-delta_max, delta_max] box:
        # u_cmd = beta * u_nom + PU(u_corr - beta * u_nom)
        # where u_nom = u_pol
        diff = u_corr - (beta * u_pol)
        pu_diff = np.clip(diff, -self.cfg.delta_max, self.cfg.delta_max)
        u_cmd = (beta * u_pol) + pu_diff

        # Cache u_cmd for next predictor cycle
        self.u_cmd_prev = u_cmd.copy()

        return u_cmd

    def get_state(self) -> dict:
        """Diagnostic state inspection."""
        return {
            "p_hat": self.p_hat.copy(),
            "sigma_hat": self.sigma_hat.copy(),
            "sigma_filtered": self.sigma_filtered.copy(),
            "u_cmd_prev": self.u_cmd_prev.copy(),
        }
