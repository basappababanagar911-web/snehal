"""
ros_compat.py
=============
ROS 2 Compatibility, Mathematical Conversions, and Interface Helpers.

Provides:
1. Rigorous 3D Quaternion <-> 2D Planar Euler Yaw conversions.
2. Velocity & Steering <-> geometry_msgs/Twist transformations.
3. Fallback message stubs for headless non-ROS execution and continuous integration.
"""

import math
from typing import Tuple, List, Optional, Any

# Check if ROS 2 rclpy and core interfaces are available
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Pose, PoseStamped, Point, Quaternion, Vector3
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Header
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    rclpy = None
    Node = object
    Twist = None
    Pose = None
    PoseStamped = None
    Point = None
    Quaternion = None
    Vector3 = None
    Odometry = None
    Path = None
    LaserScan = None
    String = None
    Header = None


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert a planar 2D yaw angle (radians) to a unit quaternion (x, y, z, w).
    For planar AGV motion: roll=0, pitch=0, yaw=psi.
    q_x = 0
    q_y = 0
    q_z = sin(yaw / 2)
    q_w = cos(yaw / 2)
    """
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Extract planar yaw angle (radians) from a unit quaternion (x, y, z, w).
    Formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def steering_to_yaw_rate(v: float, delta: float, wheelbase: float = 1.2) -> float:
    """
    Convert longitudinal velocity (m/s) and front steering angle (rad)
    to chassis rotational yaw rate omega (rad/s) via bicycle kinematics:
    omega = (v / L) * tan(delta)
    """
    return (v / max(wheelbase, 1e-4)) * math.tan(delta)


def yaw_rate_to_steering(v: float, omega: float, wheelbase: float = 1.2, max_delta: float = 0.5236) -> float:
    """
    Convert longitudinal velocity (m/s) and chassis yaw rate omega (rad/s)
    to front steering angle delta (rad). Clamped to [-max_delta, max_delta].
    delta = atan2(omega * L, v)
    """
    if abs(v) < 1e-3:
        return 0.0
    val = (omega * wheelbase) / v
    delta = math.atan(val)
    return max(-max_delta, min(max_delta, delta))


# Fallback Mock / Standalone Message Classes for Testing without ROS 2 binary
class MockHeader:
    def __init__(self, frame_id: str = "map", stamp: Any = None):
        self.frame_id = frame_id
        self.stamp = stamp or 0.0

class MockVector3:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

class MockPoint:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

class MockQuaternion:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.w = float(w)

class MockPose:
    def __init__(self, position: Optional[MockPoint] = None, orientation: Optional[MockQuaternion] = None):
        self.position = position or MockPoint()
        self.orientation = orientation or MockQuaternion()

class MockPoseStamped:
    def __init__(self, header: Optional[MockHeader] = None, pose: Optional[MockPose] = None):
        self.header = header or MockHeader()
        self.pose = pose or MockPose()

class MockTwist:
    def __init__(self, linear: Optional[MockVector3] = None, angular: Optional[MockVector3] = None):
        self.linear = linear or MockVector3()
        self.angular = angular or MockVector3()

class MockOdometry:
    def __init__(self):
        self.header = MockHeader("odom")
        self.child_frame_id = "base_link"
        self.pose = type("PoseWithCovariance", (), {"pose": MockPose()})()
        self.twist = type("TwistWithCovariance", (), {"twist": MockTwist()})()

class MockPath:
    def __init__(self):
        self.header = MockHeader("map")
        self.poses: List[MockPoseStamped] = []

class MockLaserScan:
    def __init__(self):
        self.header = MockHeader("laser_frame")
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)
        self.time_increment = 0.0
        self.scan_time = 0.05
        self.range_min = 0.1
        self.range_max = 20.0
        self.ranges: List[float] = []

class MockString:
    def __init__(self, data: str = ""):
        self.data = data
