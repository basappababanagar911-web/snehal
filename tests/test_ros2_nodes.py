"""
test_ros2_nodes.py
==================
Comprehensive Unit and Integration Tests for SteadyPath ROS 2 Deployment Stack.

Validates:
1. Coordinate transformations (Quaternion <-> 2D Euler Yaw).
2. Actuator transformations (Steering angle <-> Chassis yaw rate).
3. MPC Controller Node logic (Real-time LTV-MPC solving and horizon rollout).
4. Route Planner & Dynamic Replanner Node logic (Corridor blockage detection and detour).
5. Simulation Bridge Node logic (50 Hz kinematics and 360-degree LiDAR raycasting).
6. Telemetry Logger Node integration (Snehal's 32-column streaming format & evaluation).
"""

import os
import sys
import math
import shutil
import tempfile
import unittest
import numpy as np

# Ensure project root is in python path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from steadypath_ros2.steadypath_ros2.ros_compat import (
    yaw_to_quaternion, quaternion_to_yaw,
    steering_to_yaw_rate, yaw_rate_to_steering
)
from steadypath_ros2.steadypath_ros2.mpc_node import MPCControllerLogic
from steadypath_ros2.steadypath_ros2.planner_node import PlannerLogic
from steadypath_ros2.steadypath_ros2.sim_bridge_node import SimBridgeLogic
from steadypath_ros2.steadypath_ros2.telemetry_logger_node import TelemetryLoggerLogic
from planner.route_planner import RoutePlanner


class TestROS2MathAndTransforms(unittest.TestCase):
    """Tests geometric, kinematic, and quaternion conversions."""

    def test_quaternion_yaw_roundtrip(self):
        test_angles = [0.0, math.pi / 6.0, math.pi / 4.0, math.pi / 2.0, -math.pi / 3.0, 2.5, -2.5]
        for yaw in test_angles:
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            # Verify unit quaternion property
            norm_sq = qx**2 + qy**2 + qz**2 + qw**2
            self.assertAlmostEqual(norm_sq, 1.0, places=5)
            # Verify yaw extraction
            recovered_yaw = quaternion_to_yaw(qx, qy, qz, qw)
            self.assertAlmostEqual(yaw, recovered_yaw, places=5)

    def test_steering_to_yaw_rate_conversion(self):
        wheelbase = 1.2
        v = 1.5  # m/s
        delta = math.radians(15.0)  # ~0.2618 rad

        omega = steering_to_yaw_rate(v, delta, wheelbase)
        self.assertGreater(omega, 0.0)

        # Invert back to steering angle
        recovered_delta = yaw_rate_to_steering(v, omega, wheelbase)
        self.assertAlmostEqual(delta, recovered_delta, places=5)

    def test_zero_speed_yaw_rate_edge_case(self):
        # At zero velocity, steering conversion should not divide by zero
        recovered_delta = yaw_rate_to_steering(0.0, 0.5, 1.2)
        self.assertEqual(recovered_delta, 0.0)


class TestROS2MPCNodeLogic(unittest.TestCase):
    """Tests MPC Node computation and trajectory tracking."""

    def setUp(self):
        self.logic = MPCControllerLogic(wheelbase=1.2, dt=0.05, horizon_n=15, max_v=1.8)
        planner = RoutePlanner(default_velocity=1.2)
        route = planner.plan("DEST_BAY_01")
        waypoints = [(wp.x, wp.y, wp.yaw, wp.v) for wp in route.waypoints]
        self.logic.set_reference_path(waypoints)

    def test_mpc_solve_cycle(self):
        self.logic.set_current_state(x=0.0, y=0.0, psi=0.465, v=0.5)
        v_cmd, yaw_rate, pred_traj, diag = self.logic.compute_control()

        # Check outputs
        self.assertGreater(v_cmd, 0.0)
        self.assertLessEqual(v_cmd, 1.8)
        self.assertEqual(pred_traj.shape, (16, 4))
        self.assertEqual(diag["status"], "OPTIMAL")
        self.assertLess(diag["comp_time_ms"], 15.0)  # Should solve in under 15 ms
        self.assertIn("lateral_error", diag)

    def test_empty_path_handling(self):
        logic_empty = MPCControllerLogic()
        v_cmd, yaw_rate, pred_traj, diag = logic_empty.compute_control()
        self.assertEqual(v_cmd, 0.0)
        self.assertEqual(yaw_rate, 0.0)
        self.assertEqual(diag["status"], "WAITING_FOR_PATH")


class TestROS2PlannerNodeLogic(unittest.TestCase):
    """Tests Route Planner Node and dynamic obstacle replanning."""

    def setUp(self):
        self.planner_logic = PlannerLogic(default_velocity=1.5, obstacle_detect_dist=3.5)

    def test_nominal_plan_generation(self):
        route = self.planner_logic.active_route
        self.assertIsNotNone(route)
        self.assertGreater(len(route.waypoints), 100)
        self.assertFalse(route.is_replanned)

    def test_dynamic_replan_on_obstacle(self):
        self.planner_logic.set_vehicle_pose(x=5.0, y=0.0, yaw=0.0)
        # Place obstacle in the corridor ahead: x=7.0 (dist=2.0m)
        replan_res = self.planner_logic.check_and_replan_if_blocked(obstacle_x=7.0, obstacle_y=0.2)
        self.assertIsNotNone(replan_res)
        new_route, event_data = replan_res
        self.assertTrue(new_route.is_replanned)
        self.assertEqual(event_data["event"], "DYNAMIC_REPLAN_TRIGGERED")
        self.assertEqual(event_data["blockage_x"], 7.0)

    def test_laser_scan_triggered_replan(self):
        self.planner_logic.set_vehicle_pose(x=5.0, y=0.0, yaw=0.0)
        # Create a mock 180-ray scan with an obstacle at 2.0 meters directly forward (index 90)
        num_rays = 180
        ranges = [15.0] * num_rays
        ranges[90] = 2.0  # Center ray (0 rad) has an obstacle at 2.0 m
        angle_min = -math.pi
        angle_inc = (2.0 * math.pi) / num_rays

        res = self.planner_logic.process_laser_scan(
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_inc,
            range_max=20.0,
        )
        self.assertIsNotNone(res)
        new_route, event_data = res
        self.assertTrue(new_route.is_replanned)


class TestROS2SimBridgeLogic(unittest.TestCase):
    """Tests Simulation Bridge physics and raycast LiDAR sensor."""

    def setUp(self):
        self.sim = SimBridgeLogic(dt=0.02, wheelbase=1.2, num_rays=180)

    def test_kinematic_stepping(self):
        self.sim.set_cmd_vel(linear_x=1.5, angular_z=0.1)
        snapshot1 = self.sim.step()
        self.assertGreater(snapshot1.vehicle_v, 0.0)
        snapshot2 = self.sim.step()
        self.assertGreater(snapshot2.vehicle_x, 0.0)
        self.assertEqual(snapshot2.step, 2)

    def test_lidar_raycasting_and_boundaries(self):
        ranges = self.sim.compute_lidar_ranges(veh_x=0.0, veh_y=0.0, veh_yaw=0.0)
        self.assertEqual(len(ranges), 180)
        for r in ranges:
            self.assertGreaterEqual(r, 0.1)
            self.assertLessEqual(r, 20.0)

    def test_obstacle_detection_in_lidar(self):
        # Add an obstacle directly in front of the vehicle at x=3.0, y=0.0
        self.sim.env.add_static_obstacle(x=3.0, y=0.0, radius=0.4, obs_id="test_box")
        ranges = self.sim.compute_lidar_ranges(veh_x=0.0, veh_y=0.0, veh_yaw=0.0)
        # Front center ray is index 90 (-pi to +pi with 180 rays -> 90 is 0 rad)
        center_range = ranges[90]
        # Sensor is at x=0.4, obstacle center at 3.0, radius 0.4 -> distance ~ 3.0 - 0.4 - 0.4 = 2.2m
        self.assertAlmostEqual(center_range, 2.2, delta=0.2)


class TestROS2TelemetryLoggerLogic(unittest.TestCase):
    """Tests Telemetry Logger logic and Snehal's contract integration."""

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.logger_logic = TelemetryLoggerLogic(
            output_dir=self.temp_dir,
            run_name="test_ros2_run"
        )

    def tearDown(self):
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_telemetry_recording_and_metric_evaluation(self):
        # Simulate 10 timesteps of data
        for i in range(10):
            self.logger_logic.update_odom(
                x=i * 0.1, y=math.sin(i * 0.1), yaw=0.1, v=1.2, omega=0.05
            )
            self.logger_logic.update_control_status({
                "lateral_error": 0.005,
                "heading_error": 0.002,
                "comp_time_ms": 3.5,
                "accel_cmd": 0.1,
                "steering_cmd_rad": 0.02,
                "status": "OPTIMAL"
            })
            rec = self.logger_logic.record_step()
            self.assertEqual(rec.step, i)

        metrics = self.logger_logic.finalize()
        self.assertIn("lat_error_mean", metrics)
        self.assertIn("mpc_time_mean_ms", metrics)
        self.assertLess(metrics["lat_error_mean"], 0.05)


if __name__ == "__main__":
    unittest.main()
