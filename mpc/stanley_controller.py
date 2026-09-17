"""
stanley_controller.py
=====================
Baseline Stanley Tracking Controller (Day 11 Comparison Baseline)

Implements the classical Stanley geometric steering control law:
    delta(t) = e_psi + arctan(k * e_lat / (v + epsilon))
with longitudinal PID speed control.

Used as the benchmark against SteadyPath MPC.
"""

import time
import math
import numpy as np
from typing import Tuple, List


def normalize_angle(angle: float) -> float:
    """Normalizes angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class StanleyController:
    """Baseline Stanley trajectory tracking controller."""

    def __init__(
        self,
        wheelbase: float = 1.2,
        dt: float = 0.05,
        k_cross_track: float = 0.95,
        k_softening: float = 0.20,
        kp_v: float = 1.4,
        ki_v: float = 0.1,
        kd_v: float = 0.05
    ):
        self.L = wheelbase
        self.dt = dt
        self.k = k_cross_track
        self.epsilon = k_softening

        # Longitudinal PID gains
        self.kp_v = kp_v
        self.ki_v = ki_v
        self.kd_v = kd_v

        self._v_integral = 0.0
        self._prev_v_error = 0.0

        # Physical limits
        self.steer_max = math.radians(35.0)
        self.accel_min = -2.0
        self.accel_max = 1.5

    def solve(
        self,
        current_state: np.ndarray,
        reference_target: np.ndarray
    ) -> Tuple[float, float, float, int, List[List[float]]]:
        """
        Calculates Stanley steering and longitudinal acceleration.
        
        Args:
            current_state: [x, y, psi, v]
            reference_target: [x_ref, y_ref, psi_ref, v_ref]
            
        Returns:
            accel: Applied acceleration in m/s^2
            steer: Applied steering angle in rad
            comp_time_ms: Computation time
            status: 1 (always succeeds analytically)
            predicted_states: Empty list (Stanley has no predictive horizon)
        """
        t_start = time.perf_counter()

        x, y, psi, v = current_state
        x_ref, y_ref, psi_ref, v_ref = reference_target

        # Front axle position
        x_front = x + self.L * math.cos(psi)
        y_front = y + self.L * math.sin(psi)

        # Cross-track error at front axle
        dx = x_front - x_ref
        dy = y_front - y_ref
        e_lat = -math.sin(psi_ref) * dx + math.cos(psi_ref) * dy

        # Heading error
        e_yaw = normalize_angle(psi - psi_ref)

        # Stanley steering law
        steer_cross = math.atan2(-self.k * e_lat, max(self.epsilon, v))
        steer = -e_yaw + steer_cross
        steer = max(-self.steer_max, min(self.steer_max, steer))

        # Longitudinal PID
        e_v = v_ref - v
        self._v_integral += e_v * self.dt
        self._v_integral = max(-1.0, min(1.0, self._v_integral))  # Anti-windup
        v_derivative = (e_v - self._prev_v_error) / self.dt if self.dt > 0 else 0.0
        self._prev_v_error = e_v

        accel = self.kp_v * e_v + self.ki_v * self._v_integral + self.kd_v * v_derivative
        accel = max(self.accel_min, min(self.accel_max, accel))

        t_end = time.perf_counter()
        comp_time_ms = (t_end - t_start) * 1000.0

        return accel, steer, round(comp_time_ms, 3), 1, []

    def reset(self):
        """Resets PID integrator memory."""
        self._v_integral = 0.0
        self._prev_v_error = 0.0
