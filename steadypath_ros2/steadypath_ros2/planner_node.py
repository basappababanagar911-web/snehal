"""
planner_node.py
===============
SteadyPath Global Route Planner & Dynamic Replanner Node (R.P. Singh's Module).

Subscribes:
- /goal_pose (geometry_msgs/PoseStamped): RViz2 Goal Pose or dispatch destination.
- /scan (sensor_msgs/LaserScan): Forward safety laser scan for dynamic obstruction detection.
- /odom (nav_msgs/Odometry): AGV position feedback for dynamic replanning triggers.

Publishes:
- /global_plan (nav_msgs/Path): Densified, curvature-continuous reference trajectory.
- /steadypath/replan_event (std_msgs/String): Replanning trigger alerts with blockage location.
"""

import sys
import os
import json
import math
import numpy as np
from typing import Optional, Tuple, List, Dict, Any

# Ensure project root is importable for planner/
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from planner.route_planner import RoutePlanner, PlannedRoute, Waypoint
try:
    from steadypath_ros2.ros_compat import (
        HAS_ROS2, yaw_to_quaternion, quaternion_to_yaw,
        MockOdometry, MockPath, MockPoseStamped, MockPoint, MockQuaternion,
        MockHeader, MockString, MockLaserScan
    )
except ImportError:
    from steadypath_ros2.steadypath_ros2.ros_compat import (
        HAS_ROS2, yaw_to_quaternion, quaternion_to_yaw,
        MockOdometry, MockPath, MockPoseStamped, MockPoint, MockQuaternion,
        MockHeader, MockString, MockLaserScan
    )

if HAS_ROS2:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped, Point, Quaternion
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Header
else:
    Node = object
    Odometry = MockOdometry
    Path = MockPath
    PoseStamped = MockPoseStamped
    Point = MockPoint
    Quaternion = MockQuaternion
    LaserScan = MockLaserScan
    String = MockString
    Header = MockHeader


class PlannerLogic:
    """
    Pure Python algorithmic logic for SteadyPath Global Planner & Replanner.
    """

    def __init__(self, default_velocity: float = 1.5, obstacle_detect_dist: float = 3.5):
        self.planner = RoutePlanner(default_velocity=default_velocity)
        self.default_velocity = default_velocity
        self.obstacle_detect_dist = obstacle_detect_dist

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.active_destination = "DEST_BAY_01"
        self.active_route: Optional[PlannedRoute] = None
        self.replanned_blockages: List[Tuple[float, float]] = []

        # Initial nominal plan
        self.generate_nominal_plan(self.active_destination)

    def generate_nominal_plan(self, destination_id: str = "DEST_BAY_01") -> PlannedRoute:
        self.active_destination = destination_id
        self.active_route = self.planner.plan(destination_id=destination_id)
        return self.active_route

    def set_vehicle_pose(self, x: float, y: float, yaw: float):
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

    def set_goal_from_coordinates(self, gx: float, gy: float) -> PlannedRoute:
        """Find closest named bay or plan custom to coordinate."""
        best_bay = "DEST_BAY_01"
        min_dist = float("inf")
        for bay_name, (bx, by) in RoutePlanner.DESTINATIONS.items():
            dist = math.hypot(gx - bx, gy - by)
            if dist < min_dist:
                min_dist = dist
                best_bay = bay_name
        self.active_destination = best_bay
        return self.generate_nominal_plan(best_bay)

    def check_and_replan_if_blocked(
        self,
        obstacle_x: float,
        obstacle_y: float
    ) -> Optional[Tuple[PlannedRoute, Dict[str, Any]]]:
        """
        Evaluate if an obstacle blocks the corridor ahead of the AGV.
        If blocked, triggers smooth Gaussian detour around the blockage.
        """
        if self.active_route is None:
            return None

        # Check distance ahead
        dx = obstacle_x - self.current_x
        dy = obstacle_y - self.current_y
        dist = math.hypot(dx, dy)

        # Only replan if obstacle is ahead within detection corridor (e.g. 0.5m to 4.0m)
        if 0.5 < dx < self.obstacle_detect_dist and abs(dy) < 1.8:
            # Check if we already replanned for this vicinity
            for bx, by in self.replanned_blockages:
                if math.hypot(obstacle_x - bx, obstacle_y - by) < 1.5:
                    return None  # Already avoided

            self.replanned_blockages.append((obstacle_x, obstacle_y))
            new_route = self.planner.plan(
                destination_id=self.active_destination,
                replan_blockage=(obstacle_x, obstacle_y),
            )
            self.active_route = new_route

            event_data = {
                "event": "DYNAMIC_REPLAN_TRIGGERED",
                "blockage_x": float(obstacle_x),
                "blockage_y": float(obstacle_y),
                "replan_count": self.planner.replan_count,
                "total_path_length": new_route.total_length,
                "destination_id": self.active_destination,
            }
            return new_route, event_data

        return None

    def process_laser_scan(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_max: float = 20.0
    ) -> Optional[Tuple[PlannedRoute, Dict[str, Any]]]:
        """
        Inspect 2D Lidar scan for forward obstacles in AGV body frame,
        project them to map frame, and check for replanning triggers.
        """
        if not ranges:
            return None

        num_ranges = len(ranges)
        # Search narrow forward arc [-30 deg, +30 deg]
        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.2 or r > range_max:
                continue

            angle = angle_min + i * angle_increment
            if -math.radians(35) <= angle <= math.radians(35):
                # Obstacle in body frame
                x_b = r * math.cos(angle)
                y_b = r * math.sin(angle)

                # Transform to map frame
                x_m = self.current_x + x_b * math.cos(self.current_yaw) - y_b * math.sin(self.current_yaw)
                y_m = self.current_y + x_b * math.sin(self.current_yaw) + y_b * math.cos(self.current_yaw)

                result = self.check_and_replan_if_blocked(x_m, y_m)
                if result is not None:
                    return result

        return None

    def to_nav_path_poses(self) -> List[Tuple[float, float, float, float]]:
        """Extract [(x, y, qz, qw)] list for ROS 2 Path."""
        if not self.active_route:
            return []
        poses = []
        for wp in self.active_route.waypoints:
            qx, qy, qz, qw = yaw_to_quaternion(wp.yaw)
            poses.append((wp.x, wp.y, qz, qw))
        return poses


class SteadyPathPlannerNode(Node):
    """
    ROS 2 Node wrapper for SteadyPath Global Route Planner & Replanner.
    """

    def __init__(self):
        super().__init__("steadypath_planner_node")

        self.declare_parameter("default_velocity", 1.5)
        self.declare_parameter("obstacle_detect_dist", 3.5)
        self.declare_parameter("destination_id", "DEST_BAY_01")
        self.declare_parameter("plan_publish_frequency", 2.0)  # Hz

        v_def = self.get_parameter("default_velocity").value
        obs_dist = self.get_parameter("obstacle_detect_dist").value
        dest_id = self.get_parameter("destination_id").value
        pub_freq = self.get_parameter("plan_publish_frequency").value

        self.logic = PlannerLogic(default_velocity=v_def, obstacle_detect_dist=obs_dist)
        self.logic.generate_nominal_plan(dest_id)

        # Publishers
        self.plan_pub = self.create_publisher(Path, "/global_plan", 10)
        self.replan_event_pub = self.create_publisher(String, "/steadypath/replan_event", 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        # Periodic publisher timer to keep global plan latched/refreshed
        self.timer = self.create_timer(1.0 / pub_freq, self.publish_plan)

        self.get_logger().info(
            f"SteadyPath Planner Node Initialized. Destination: {dest_id}, Velocity: {v_def} m/s"
        )
        self.publish_plan()

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.logic.set_vehicle_pose(x, y, yaw)

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        result = self.logic.process_laser_scan(
            ranges=ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_max=msg.range_max,
        )
        if result is not None:
            _, event_data = result
            self.get_logger().warn(
                f"[REPLAN] Dynamic obstacle detected at ({event_data['blockage_x']:.2f}, "
                f"{event_data['blockage_y']:.2f})! Replanning detour path."
            )
            str_msg = String()
            str_msg.data = json.dumps(event_data)
            self.replan_event_pub.publish(str_msg)
            self.publish_plan()

    def goal_callback(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f"Received new goal request: ({gx:.2f}, {gy:.2f})")
        self.logic.set_goal_from_coordinates(gx, gy)
        self.publish_plan()

    def publish_plan(self):
        if not self.logic.active_route:
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for wp in self.logic.active_route.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(wp.x)
            ps.pose.position.y = float(wp.y)
            qx, qy, qz, qw = yaw_to_quaternion(wp.yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_msg.poses.append(ps)

        self.plan_pub.publish(path_msg)


def main(args=None):
    if not HAS_ROS2:
        print("[ERROR] rclpy is not installed on this system. Please run in a ROS 2 container.")
        sys.exit(1)
    rclpy.init(args=args)
    node = SteadyPathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
