"""
kinematic_model.py
==================
SteadyPath Kinematic Bicycle Model (MPC & Control Lead)

Represents front-steered Autonomous Guided Vehicle (AGV) kinematics:
    x_dot = v * cos(psi)
    y_dot = v * sin(psi)
    psi_dot = (v / L) * tan(delta)
    v_dot = a

Provides continuous and discrete state propagation, plus Jacobians
for Linear Time-Varying (LTV) MPC optimization.
"""

import math
import numpy as np
from typing import Tuple


class KinematicBicycleModel:
    """Kinematic bicycle model parameterized by wheelbase L."""

    def __init__(self, wheelbase: float = 1.2, dt: float = 0.05):
        self.L = wheelbase
        self.dt = dt

        # State bounds
        self.v_min = 0.0
        self.v_max = 2.5
        self.steer_max = math.radians(35.0)  # +/- 35 deg
        self.accel_min = -2.0
        self.accel_max = 1.5

        # Rate bounds
        self.max_steer_rate = math.radians(45.0)  # 45 deg/s
        self.max_jerk = 2.5  # m/s^3

    def step(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """
        Integrates kinematics forward by dt using Runge-Kutta 4 (RK4).
        state: [x, y, psi, v]
        control: [a, delta]
        """
        dt = self.dt
        k1 = self._derivative(state, control)
        k2 = self._derivative(state + 0.5 * dt * k1, control)
        k3 = self._derivative(state + 0.5 * dt * k2, control)
        k4 = self._derivative(state + dt * k3, control)

        next_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        # Normalize yaw to [-pi, pi]
        next_state[2] = math.atan2(math.sin(next_state[2]), math.cos(next_state[2]))
        # Clamp speed
        next_state[3] = max(self.v_min, min(self.v_max, next_state[3]))
        return next_state

    def _derivative(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """Continuous derivative f(x, u)."""
        x, y, psi, v = state
        a, delta = control
        # Clamp steering to physical limits
        delta = max(-self.steer_max, min(self.steer_max, delta))
        return np.array([
            v * math.cos(psi),
            v * math.sin(psi),
            (v / self.L) * math.tan(delta),
            a
        ])

    def linearize(self, state_ref: np.ndarray, control_ref: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Computes LTV Jacobians A_k, B_k and affine offset d_k:
            x_{k+1} approx A_k * x_k + B_k * u_k + d_k
        """
        x, y, psi, v = state_ref
        a, delta = control_ref
        dt = self.dt
        L = self.L

        delta = max(-self.steer_max + 1e-4, min(self.steer_max - 1e-4, delta))
        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        tan_del = math.tan(delta)
        cos_del = math.cos(delta)

        # State transition matrix A
        A = np.eye(4)
        A[0, 2] = -v * sin_psi * dt
        A[0, 3] = cos_psi * dt
        A[1, 2] = v * cos_psi * dt
        A[1, 3] = sin_psi * dt
        A[2, 3] = (tan_del / L) * dt

        # Control input matrix B
        B = np.zeros((4, 2))
        B[2, 1] = (v / (L * (cos_del ** 2))) * dt
        B[3, 0] = dt

        # Affine remainder: d = f(x_ref, u_ref)*dt - A*x_ref - B*u_ref + x_ref
        f_val = self._derivative(state_ref, control_ref) * dt
        d = f_val - A @ state_ref - B @ control_ref + state_ref

        return A, B, d
