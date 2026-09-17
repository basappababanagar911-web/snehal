"""
test_all.py
===========
SteadyPath Automated Test & Verification Suite

Verifies:
1. Kinematic Vehicle Model (RK4 integration, bounds, Jacobians)
2. SteadyPath MPC Controller (Convergence, rate limits, horizon output)
3. Stanley Controller (Baseline calculations and limits)
4. Route Planner (Spline generation, curvature, dynamic replanning)
5. Telemetry Logger & Evaluator (Contract compliance, Frenet errors, metrics)
6. Simulation Environment (Obstacle clearance, collision detection)
"""

import os
import sys
import math
import unittest
import numpy as np

# Ensure root and analysis src are in sys.path
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, root_dir)
sys.path.insert(0, os.path.join(root_dir, "analysis", "src"))

from mpc.kinematic_model import KinematicBicycleModel
from mpc.mpc_controller import SteadyPathMPC
from mpc.stanley_controller import StanleyController
from planner.route_planner import RoutePlanner
from simulation.warehouse_env import WarehouseEnvironment
from analysis.src.evaluator import (
    compute_lateral_error,
    compute_heading_error,
    compute_rms,
    SteadyPathEvaluator
)
from analysis.src.data_logger import STANDARD_CSV_COLUMNS, TelemetryRecord


class TestKinematicModel(unittest.TestCase):
    """Verifies vehicle kinematic bicycle model."""

    def setUp(self):
        self.model = KinematicBicycleModel(wheelbase=1.2, dt=0.05)

    def test_straight_line_integration(self):
        """Vehicle with zero steering and constant acceleration moves in straight line."""
        state = np.array([0.0, 0.0, 0.0, 1.0])
        control = np.array([0.0, 0.0])  # zero accel, zero steer
        next_state = self.model.step(state, control)

        # Expected x = x0 + v * dt = 0 + 1.0 * 0.05 = 0.05
        self.assertAlmostEqual(next_state[0], 0.05, places=4)
        self.assertAlmostEqual(next_state[1], 0.0, places=4)
        self.assertAlmostEqual(next_state[2], 0.0, places=4)
        self.assertAlmostEqual(next_state[3], 1.0, places=4)

    def test_turning_kinematics(self):
        """Positive steering turns the vehicle counter-clockwise (positive yaw)."""
        state = np.array([0.0, 0.0, 0.0, 1.5])
        control = np.array([0.0, math.radians(15.0)])
        next_state = self.model.step(state, control)
        self.assertGreater(next_state[2], 0.0)

    def test_steering_saturation(self):
        """Steering command beyond +/- 35 deg is physically clamped."""
        state = np.array([0.0, 0.0, 0.0, 1.0])
        control = np.array([0.0, math.radians(60.0)])  # Excessive steer
        next_state = self.model.step(state, control)
        # Yaw rate should correspond to max 35 deg
        max_yaw_rate = (1.0 / 1.2) * math.tan(math.radians(35.0))
        actual_yaw_rate = next_state[2] / 0.05
        self.assertAlmostEqual(actual_yaw_rate, max_yaw_rate, places=2)

    def test_linearization_dimensions(self):
        """Jacobians A, B must match state dimension 4 and control dimension 2."""
        state = np.array([10.0, 2.0, 0.3, 1.2])
        control = np.array([0.1, 0.05])
        A, B, d = self.model.linearize(state, control)
        self.assertEqual(A.shape, (4, 4))
        self.assertEqual(B.shape, (4, 2))
        self.assertEqual(d.shape, (4,))


class TestControllers(unittest.TestCase):
    """Verifies MPC and Stanley controllers."""

    def test_mpc_solve_convergence(self):
        """MPC solver converges within budget and outputs valid horizon trajectory."""
        mpc = SteadyPathMPC(wheelbase=1.2, dt=0.05, horizon=15)
        current_state = np.array([0.0, 0.05, 0.02, 1.4])
        # Simple straight reference trajectory
        ref_traj = np.zeros((16, 4))
        ref_traj[:, 0] = np.linspace(0.0, 3.0, 16)
        ref_traj[:, 3] = 1.5

        # Initial warm-up solve
        mpc.solve(current_state, ref_traj)

        # Subsequent warm-started solve (real-time loop condition)
        accel, steer, comp_time, status, pred_traj = mpc.solve(current_state, ref_traj)
        self.assertEqual(status, 1)
        self.assertLessEqual(comp_time, 50.0)  # Real-time requirement (dt = 50ms)
        self.assertEqual(len(pred_traj), 15)
        self.assertGreaterEqual(accel, -2.0)
        self.assertLessEqual(accel, 1.5)
        self.assertGreaterEqual(steer, -math.radians(35.0))
        self.assertLessEqual(steer, math.radians(35.0))

    def test_stanley_solve(self):
        """Stanley controller computes valid control signals."""
        stanley = StanleyController(wheelbase=1.2, dt=0.05)
        current_state = np.array([0.0, 0.1, 0.0, 1.0])
        ref_target = np.array([0.0, 0.0, 0.0, 1.5])

        accel, steer, comp_time, status, _ = stanley.solve(current_state, ref_target)
        self.assertEqual(status, 1)
        self.assertLessEqual(comp_time, 5.0)
        # Vehicle is to the left of path (y=0.1 > y_ref=0), steer should steer right (negative)
        self.assertLess(steer, 0.0)


class TestPlanner(unittest.TestCase):
    """Verifies R.P. Singh's route planner."""

    def setUp(self):
        self.planner = RoutePlanner(default_velocity=1.5)

    def test_nominal_route_generation(self):
        """Planner generates continuous route to target destination."""
        route = self.planner.plan(destination_id="DEST_BAY_01")
        self.assertGreater(len(route.waypoints), 100)
        self.assertAlmostEqual(route.waypoints[-1].x, 30.0, places=1)
        self.assertGreater(route.total_length, 30.0)
        self.assertFalse(route.is_replanned)

    def test_dynamic_replan_detour(self):
        """Planner generates collision-free detour when blockage is specified."""
        nominal_route = self.planner.plan(destination_id="DEST_BAY_01")
        replan_route = self.planner.plan(destination_id="DEST_BAY_01", replan_blockage=(14.0, 0.0))

        self.assertTrue(replan_route.is_replanned)
        self.assertEqual(replan_route.replan_count, 1)
        # Detour route must be slightly longer than nominal
        self.assertGreater(replan_route.total_length, nominal_route.total_length)

    def test_reference_window_extraction(self):
        """Reference window extracted has correct shape (N+1, 4)."""
        self.planner.plan(destination_id="DEST_BAY_01")
        window = self.planner.get_reference_window(current_x=5.0, horizon_steps=15, dt=0.05)
        self.assertEqual(window.shape, (16, 4))
        # Speeds should match nominal cruise speed
        self.assertAlmostEqual(window[0, 3], 1.5, places=1)


class TestEvaluatorAndLogger(unittest.TestCase):
    """Verifies Snehal's data contracts and mathematical error formulas."""

    def test_standard_columns_count(self):
        """CSV standard column contract must have exactly 32 columns."""
        self.assertEqual(len(STANDARD_CSV_COLUMNS), 32)

    def test_lateral_error_formula(self):
        """Frenet lateral deviation math: positive = left of path."""
        # Vehicle at (0, 1), reference at (0, 0) with yaw_ref = 0 (facing +X)
        e_lat = compute_lateral_error(x=0.0, y=1.0, x_ref=0.0, y_ref=0.0, yaw_ref=0.0)
        self.assertAlmostEqual(e_lat, 1.0, places=4)

        # Reference facing +Y (yaw_ref = pi/2)
        # Vehicle at (1, 0), reference at (0, 0) -> vehicle is to the right of path
        e_lat_right = compute_lateral_error(x=1.0, y=0.0, x_ref=0.0, y_ref=0.0, yaw_ref=math.pi / 2.0)
        self.assertAlmostEqual(e_lat_right, -1.0, places=4)

    def test_heading_error_normalization(self):
        """Heading error must wrap properly within [-pi, pi]."""
        # Yaw = 3.10 rad, Yaw_ref = -3.10 rad -> difference is 6.20 rad (~2pi) -> near 0
        err = compute_heading_error(yaw=3.10, yaw_ref=-3.10)
        self.assertAlmostEqual(err, 3.10 - (-3.10) - 2.0 * math.pi, places=4)

    def test_rms_calculation(self):
        """RMS of [3, 4] is sqrt((9+16)/2) = sqrt(12.5) ~= 3.5355."""
        val = compute_rms([3.0, 4.0])
        self.assertAlmostEqual(val, math.sqrt(12.5), places=4)


class TestSimulationEnvironment(unittest.TestCase):
    """Verifies Sheshank's simulation physics and obstacle detection."""

    def setUp(self):
        self.env = WarehouseEnvironment(dt=0.05, wheelbase=1.2, vehicle_radius=0.35)

    def test_obstacle_clearance_and_collision(self):
        """Clearance calculation and collision detection trigger accurately."""
        self.env.reset(initial_state=np.array([0.0, 0.0, 0.0, 0.0]))
        self.env.add_static_obstacle(x=5.0, y=0.0, radius=0.5, obs_id="test_obs")

        # Step towards obstacle
        snapshot = self.env.step(accel=1.0, steer=0.0)
        self.assertFalse(snapshot.collision_detected)
        # Distance = 5.0 - (0.5 + 0.35) - x_pos
        expected_clearance = 5.0 - (0.5 + 0.35) - snapshot.vehicle_x
        self.assertAlmostEqual(snapshot.min_clearance, expected_clearance, places=3)


if __name__ == "__main__":
    unittest.main()
