"""
mpc_node.py
===========
SteadyPath ROS 2 Model Predictive Control (MPC) Node.

Subscribes:
- /odom (nav_msgs/Odometry): AGV pose and velocity feedback.
- /global_plan (nav_msgs/Path): Reference trajectory from planner.

Publishes:
- /cmd_vel (geometry_msgs/Twist): High-precision velocity and steering yaw rate.
- /mpc_predicted_path (nav_msgs/Path): Visual predicted horizon rollout (N=15) for RViz2.
- /steadypath/control_status (std_msgs/String): Real-time solver diagnostics and tracking errors.
"""

import sys
import os
import json
import time
import math
import numpy as np
from typing import List, Optional, Tuple, Dict, Any

# Ensure project root is importable for mpc/
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from mpc.mpc_controller import SteadyPathMPC
try:
    from steadypath_ros2.ros_compat import (
        HAS_ROS2, yaw_to_quaternion, quaternion_to_yaw,
        steering_to_yaw_rate, MockOdometry, MockPath,
        MockTwist, MockPoseStamped, MockPoint, MockQuaternion, MockHeader, MockString
    )
except ImportError:
    from steadypath_ros2.steadypath_ros2.ros_compat import (
        HAS_ROS2, yaw_to_quaternion, quaternion_to_yaw,
        steering_to_yaw_rate, MockOdometry, MockPath,
        MockTwist, MockPoseStamped, MockPoint, MockQuaternion, MockHeader, MockString
    )

if HAS_ROS2:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
    from std_msgs.msg import String, Header
else:
    Node = object
    Odometry = MockOdometry
    Path = MockPath
    Twist = MockTwist
    PoseStamped = MockPoseStamped
    Point = MockPoint
    Quaternion = MockQuaternion
    String = MockString
    Header = MockHeader


class MPCControllerLogic:
    """
    Pure Python algorithmic logic for SteadyPath ROS 2 MPC Node.
    Encapsulates trajectory sampling, MPC solving, and message conversion
    without requiring active ROS 2 middleware daemon.
    """

    def __init__(
        self,
        wheelbase: float = 1.2,
        dt: float = 0.05,
        horizon_n: int = 15,
        max_v: float = 1.5,
        max_accel: float = 1.0,
        max_steer: float = 0.5236,
        q_weights: Optional[List[float]] = None,
        r_weights: Optional[List[float]] = None,
        rd_weights: Optional[List[float]] = None,
    ):
        self.wheelbase = wheelbase
        self.dt = dt
        self.horizon_n = horizon_n
        self.max_v = max_v
        self.max_accel = max_accel
        self.max_steer = max_steer

        q = q_weights or [3.0, 5.5, 3.0, 1.8]
        r = r_weights or [0.15, 0.40]
        rd = rd_weights or [0.25, 0.90]

        self.mpc = SteadyPathMPC(
            wheelbase=self.wheelbase,
            dt=self.dt,
            horizon=self.horizon_n,
            w_x=q[0],
            w_y=q[1],
            w_yaw=q[2],
            w_v=q[3],
            w_accel=r[0],
            w_steer=r[1],
            w_daccel=rd[0],
            w_dsteer=rd[1],
        )

        self.current_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, psi, v]
        self.reference_path: List[Tuple[float, float, float, float]] = []  # [(x, y, psi, v)]
        self.last_steer = 0.0
        self.last_accel = 0.0

    def set_current_state(self, x: float, y: float, psi: float, v: float):
        self.current_state = np.array([x, y, psi, v])

    def set_reference_path(self, waypoints: List[Tuple[float, float, float, float]]):
        """Set reference path where each waypoint is (x, y, psi, v)."""
        self.reference_path = waypoints

    def find_nearest_waypoint_index(self) -> int:
        if not self.reference_path:
            return 0
        px, py = self.current_state[0], self.current_state[1]
        best_idx = 0
        min_dist_sq = float("inf")
        for i, wp in enumerate(self.reference_path):
            d_sq = (wp[0] - px) ** 2 + (wp[1] - py) ** 2
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                best_idx = i
        return best_idx

    def compute_control(self) -> Tuple[float, float, np.ndarray, Dict[str, Any]]:
        """
        Executes one MPC optimization cycle.
        Returns:
            linear_velocity_cmd: float (m/s)
            yaw_rate_cmd: float (rad/s)
            predicted_horizon: np.ndarray shape (N+1, 4)
            diagnostics: Dict[str, Any]
        """
        if len(self.reference_path) == 0:
            # No reference trajectory received yet
            return 0.0, 0.0, np.zeros((self.horizon_n + 1, 4)), {
                "status": "WAITING_FOR_PATH",
                "lateral_error": 0.0,
                "heading_error": 0.0,
                "comp_time_ms": 0.0,
            }

        start_idx = self.find_nearest_waypoint_index()
        num_wp = len(self.reference_path)

        # Build horizon reference array of shape (N+1, 4)
        ref_horizon = np.zeros((self.horizon_n + 1, 4))
        for k in range(self.horizon_n + 1):
            idx = min(start_idx + k, num_wp - 1)
            ref_horizon[k, :] = self.reference_path[idx]

        # Tracking errors at current step
        ref_curr = ref_horizon[0]
        dx = self.current_state[0] - ref_curr[0]
        dy = self.current_state[1] - ref_curr[1]
        psi_ref = ref_curr[2]
        # Frenet lateral error
        lat_error = -math.sin(psi_ref) * dx + math.cos(psi_ref) * dy
        head_error = math.atan2(math.sin(self.current_state[2] - psi_ref), math.cos(self.current_state[2] - psi_ref))

        t0 = time.perf_counter()
        accel_cmd, steer_cmd, mpc_solve_time, status, pred_list = self.mpc.solve(
            self.current_state, ref_horizon
        )
        total_time_ms = (time.perf_counter() - t0) * 1000.0

        # Assemble full (N+1, 4) predicted horizon array starting at current state
        if isinstance(pred_list, list) and len(pred_list) > 0:
            full_pred_traj = np.vstack([self.current_state, np.array(pred_list)])
        else:
            full_pred_traj = np.zeros((self.horizon_n + 1, 4))

        # Command calculation: update commanded target speed
        target_v = float(np.clip(self.current_state[3] + accel_cmd * self.dt, 0.0, self.max_v))
        yaw_rate = steering_to_yaw_rate(target_v, steer_cmd, self.wheelbase)

        self.last_steer = steer_cmd
        self.last_accel = accel_cmd

        diagnostics = {
            "status": "OPTIMAL" if status == 1 else "FALLBACK",
            "lateral_error": float(lat_error),
            "heading_error": float(head_error),
            "comp_time_ms": float(total_time_ms),
            "accel_cmd": float(accel_cmd),
            "steering_cmd_rad": float(steer_cmd),
            "steering_cmd_deg": float(math.degrees(steer_cmd)),
            "v_cmd": float(target_v),
            "yaw_rate_cmd": float(yaw_rate),
        }

        return target_v, yaw_rate, full_pred_traj, diagnostics


class SteadyPathMPCNode(Node):
    """
    ROS 2 Node wrapper for SteadyPath MPC.
    """

    def __init__(self):
        super().__init__("steadypath_mpc_node")

        # Declare parameters
        self.declare_parameter("wheelbase", 1.2)
        self.declare_parameter("control_frequency", 50.0)  # Hz
        self.declare_parameter("horizon_n", 15)
        self.declare_parameter("mpc_dt", 0.05)
        self.declare_parameter("max_velocity", 1.5)
        self.declare_parameter("max_accel", 1.0)
        self.declare_parameter("max_steer_rad", 0.5236)

        wheelbase = self.get_parameter("wheelbase").value
        control_freq = self.get_parameter("control_frequency").value
        horizon_n = self.get_parameter("horizon_n").value
        mpc_dt = self.get_parameter("mpc_dt").value
        max_v = self.get_parameter("max_velocity").value
        max_accel = self.get_parameter("max_accel").value
        max_steer = self.get_parameter("max_steer_rad").value

        self.logic = MPCControllerLogic(
            wheelbase=wheelbase,
            dt=mpc_dt,
            horizon_n=horizon_n,
            max_v=max_v,
            max_accel=max_accel,
            max_steer=max_steer,
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pred_path_pub = self.create_publisher(Path, "/mpc_predicted_path", 10)
        self.status_pub = self.create_publisher(String, "/steadypath/control_status", 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.plan_sub = self.create_subscription(Path, "/global_plan", self.plan_callback, 10)

        # Periodic control timer (default: 50 Hz -> 0.02 s)
        timer_period = 1.0 / control_freq
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f"SteadyPath MPC Node Initialized (Rate: {control_freq} Hz, Horizon: {horizon_n}, Wheelbase: {wheelbase} m)"
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear.x
        self.logic.set_current_state(x, y, yaw, v)

    def plan_callback(self, msg: Path):
        waypoints = []
        for p in msg.poses:
            wx = p.pose.position.x
            wy = p.pose.position.y
            q = p.pose.orientation
            wyaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            # Default target speed of 1.2 m/s if not specified
            waypoints.append((wx, wy, wyaw, 1.2))
        self.logic.set_reference_path(waypoints)

    def control_loop(self):
        v_cmd, yaw_rate, pred_traj, diagnostics = self.logic.compute_control()

        # Publish Twist
        twist_msg = Twist()
        twist_msg.linear.x = float(v_cmd)
        twist_msg.angular.z = float(yaw_rate)
        self.cmd_vel_pub.publish(twist_msg)

        # Publish Predicted Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for row in pred_traj:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(row[0])
            ps.pose.position.y = float(row[1])
            qx, qy, qz, qw = yaw_to_quaternion(row[2])
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_msg.poses.append(ps)
        self.pred_path_pub.publish(path_msg)

        # Publish Diagnostics
        status_msg = String()
        status_msg.data = json.dumps(diagnostics)
        self.status_pub.publish(status_msg)


def main(args=None):
    if not HAS_ROS2:
        print("[ERROR] rclpy is not installed on this system. Please run in a ROS 2 container.")
        sys.exit(1)
    rclpy.init(args=args)
    node = SteadyPathMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
