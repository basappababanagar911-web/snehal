"""
sim_bridge_node.py
==================
SteadyPath Warehouse Simulation Bridge Node (B. Sheshank's Module).

Integrates the 50 Hz kinematic AGV physics, warehouse obstacles, moving forklifts,
and publishes simulated sensor streams:
- /odom (nav_msgs/Odometry): Ground-truth AGV localization.
- /scan (sensor_msgs/LaserScan): 360-degree ray-casted 2D LiDAR range stream.
- TF transforms: odom -> base_link -> laser_frame.

Subscribes:
- /cmd_vel (geometry_msgs/Twist): Actuation velocity and yaw rate commands.
"""

import sys
import os
import math
import numpy as np
from typing import List, Tuple, Dict, Optional, Any

# Ensure project root is importable for simulation/
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simulation.warehouse_env import WarehouseEnvironment, SimSensorSnapshot, ObstacleState
try:
    from steadypath_ros2.ros_compat import (
        HAS_ROS2, yaw_to_quaternion, quaternion_to_yaw,
        yaw_rate_to_steering, steering_to_yaw_rate,
        MockOdometry, MockLaserScan, MockTwist
    )
except ImportError:
    from steadypath_ros2.steadypath_ros2.ros_compat import (
        HAS_ROS2, yaw_to_quaternion, quaternion_to_yaw,
        yaw_rate_to_steering, steering_to_yaw_rate,
        MockOdometry, MockLaserScan, MockTwist
    )

if HAS_ROS2:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist, TransformStamped
    from tf2_ros import TransformBroadcaster
else:
    Node = object
    Odometry = MockOdometry
    LaserScan = MockLaserScan
    Twist = MockTwist
    TransformStamped = None
    TransformBroadcaster = None


class SimBridgeLogic:
    """
    Pure Python algorithmic logic for warehouse simulation and ray-cast LiDAR.
    """

    def __init__(
        self,
        dt: float = 0.02,
        wheelbase: float = 1.2,
        num_rays: int = 180,
        max_lidar_range: float = 20.0,
        warehouse_bounds: Tuple[float, float, float, float] = (-5.0, 45.0, -8.0, 8.0),
    ):
        self.dt = dt
        self.wheelbase = wheelbase
        self.num_rays = num_rays
        self.max_lidar_range = max_lidar_range
        self.x_min, self.x_max, self.y_min, self.y_max = warehouse_bounds

        self.env = WarehouseEnvironment(dt=self.dt, wheelbase=self.wheelbase)
        self.env.reset(np.array([0.0, 0.0, 0.465, 0.0]))

        # Target command inputs
        self.target_v = 0.0
        self.target_steer = 0.0

        # Laser geometry
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (self.angle_max - self.angle_min) / float(self.num_rays)

    def set_cmd_vel(self, linear_x: float, angular_z: float):
        """Map Twist message to vehicle controls."""
        self.target_v = float(np.clip(linear_x, 0.0, self.env.v_max))
        self.target_steer = yaw_rate_to_steering(
            v=self.target_v,
            omega=angular_z,
            wheelbase=self.wheelbase,
            max_delta=self.env.steer_max,
        )

    def step(self) -> SimSensorSnapshot:
        """Advance physics by dt."""
        curr_v = self.env.state[3]
        # P-controlled acceleration to track target velocity
        accel = (self.target_v - curr_v) / max(self.dt, 1e-3)
        accel = float(np.clip(accel, -2.0, 1.5))

        snapshot = self.env.step(accel=accel, steer=self.target_steer)
        return snapshot

    def compute_lidar_ranges(self, veh_x: float, veh_y: float, veh_yaw: float) -> List[float]:
        """
        Analytically casts rays across 360 degrees to detect warehouse boundaries
        and circular obstacle boundaries.
        """
        # LiDAR sensor positioned 0.4 m forward of vehicle center
        lx = veh_x + 0.4 * math.cos(veh_yaw)
        ly = veh_y + 0.4 * math.sin(veh_yaw)

        ranges: List[float] = []

        for i in range(self.num_rays):
            angle_rel = self.angle_min + i * self.angle_increment
            ray_theta = veh_yaw + angle_rel
            cos_a = math.cos(ray_theta)
            sin_a = math.sin(ray_theta)

            closest_d = self.max_lidar_range

            # 1. Intersection with warehouse boundary walls
            # x = x_min, x = x_max
            if abs(cos_a) > 1e-4:
                d_x1 = (self.x_min - lx) / cos_a
                if d_x1 > 0.0:
                    y_int = ly + d_x1 * sin_a
                    if self.y_min <= y_int <= self.y_max and d_x1 < closest_d:
                        closest_d = d_x1

                d_x2 = (self.x_max - lx) / cos_a
                if d_x2 > 0.0:
                    y_int = ly + d_x2 * sin_a
                    if self.y_min <= y_int <= self.y_max and d_x2 < closest_d:
                        closest_d = d_x2

            # y = y_min, y = y_max
            if abs(sin_a) > 1e-4:
                d_y1 = (self.y_min - ly) / sin_a
                if d_y1 > 0.0:
                    x_int = lx + d_y1 * cos_a
                    if self.x_min <= x_int <= self.x_max and d_y1 < closest_d:
                        closest_d = d_y1

                d_y2 = (self.y_max - ly) / sin_a
                if d_y2 > 0.0:
                    x_int = lx + d_y2 * cos_a
                    if self.x_min <= x_int <= self.x_max and d_y2 < closest_d:
                        closest_d = d_y2

            # 2. Line-circle intersection with each obstacle
            for obs in self.env.obstacles:
                # Vector from lidar origin to obstacle center
                cx = obs.x - lx
                cy = obs.y - ly

                # Projection of center along ray: t = c . d
                t = cx * cos_a + cy * sin_a
                if t > 0.0:
                    # Perpendicular distance squared: dist_sq = |c|^2 - t^2
                    dist_sq = (cx * cx + cy * cy) - (t * t)
                    r_sq = obs.radius * obs.radius
                    if dist_sq <= r_sq:
                        half_chord = math.sqrt(max(0.0, r_sq - dist_sq))
                        d_hit = t - half_chord
                        if 0.1 < d_hit < closest_d:
                            closest_d = d_hit

            ranges.append(round(float(closest_d), 3))

        return ranges


class SteadyPathSimBridgeNode(Node):
    """
    ROS 2 Node publishing simulated odometry, transforms, and laser scans.
    """

    def __init__(self):
        super().__init__("steadypath_sim_bridge_node")

        self.declare_parameter("sim_frequency", 50.0)  # Hz
        self.declare_parameter("wheelbase", 1.2)
        self.declare_parameter("scenario", "moving")

        sim_freq = self.get_parameter("sim_frequency").value
        wheelbase = self.get_parameter("wheelbase").value
        scenario = self.get_parameter("scenario").value

        dt = 1.0 / sim_freq
        self.logic = SimBridgeLogic(dt=dt, wheelbase=wheelbase)

        # Configure obstacles based on scenario
        if scenario == "replan":
            self.logic.env.add_static_obstacle(x=14.0, y=0.5, radius=0.6, obs_id="blocked_rack")
        elif scenario == "moving":
            self.logic.env.add_static_obstacle(x=10.0, y=-1.5, radius=0.5, obs_id="rack_bay1")
            self.logic.env.add_moving_obstacle(start_x=12.0, start_y=-2.5, vx=0.0, vy=0.25, radius=0.45, obs_id="crossing_forklift")

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)

        # Subscriber
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

        # TF Broadcaster
        if HAS_ROS2:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # Simulation timer loop
        self.timer = self.create_timer(dt, self.simulation_loop)

        self.get_logger().info(
            f"SteadyPath Simulation Bridge Node Initialized (Rate: {sim_freq} Hz, Scenario: {scenario})"
        )

    def cmd_callback(self, msg: Twist):
        self.logic.set_cmd_vel(msg.linear.x, msg.angular.z)

    def simulation_loop(self):
        snapshot = self.logic.step()
        now = self.get_clock().now().to_msg()

        # 1. Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = snapshot.vehicle_x
        odom_msg.pose.pose.position.y = snapshot.vehicle_y
        odom_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(snapshot.vehicle_yaw)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = snapshot.vehicle_v
        steer_rad = self.logic.target_steer
        odom_msg.twist.twist.angular.z = steering_to_yaw_rate(snapshot.vehicle_v, steer_rad, self.logic.wheelbase)

        self.odom_pub.publish(odom_msg)

        # 2. Publish TF (odom -> base_link)
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = snapshot.vehicle_x
            t.transform.translation.y = snapshot.vehicle_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        # 3. Publish LaserScan
        ranges = self.logic.compute_lidar_ranges(
            snapshot.vehicle_x, snapshot.vehicle_y, snapshot.vehicle_yaw
        )
        scan_msg = LaserScan()
        scan_msg.header.stamp = now
        scan_msg.header.frame_id = "laser_frame"
        scan_msg.angle_min = self.logic.angle_min
        scan_msg.angle_max = self.logic.angle_max
        scan_msg.angle_increment = self.logic.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = float(self.logic.dt)
        scan_msg.range_min = 0.1
        scan_msg.range_max = self.logic.max_lidar_range
        scan_msg.ranges = ranges

        self.scan_pub.publish(scan_msg)


def main(args=None):
    if not HAS_ROS2:
        print("[ERROR] rclpy is not installed on this system. Please run in a ROS 2 container.")
        sys.exit(1)
    rclpy.init(args=args)
    node = SteadyPathSimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
