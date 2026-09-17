"""
telemetry_logger_node.py
========================
SteadyPath Telemetry Logger Node (Snehal's Module Integration).

Subscribes to ROS 2 topics:
- /odom (nav_msgs/Odometry)
- /cmd_vel (geometry_msgs/Twist)
- /global_plan (nav_msgs/Path)
- /mpc_predicted_path (nav_msgs/Path)
- /steadypath/control_status (std_msgs/String)
- /steadypath/replan_event (std_msgs/String)

Enforces Snehal's 32-column telemetry standard, streaming data directly to:
- CSV: analysis/telemetry_logs/<run_id>.csv
- JSONL: analysis/telemetry_logs/<run_id>.jsonl
Computes final metrics on shutdown.
"""

import sys
import os
import json
import time
import math
from typing import Optional, List, Dict, Any

# Ensure project root is importable
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from analysis.src.data_logger import SteadyPathLogger, TelemetryRecord
from analysis.src.evaluator import SteadyPathEvaluator, RunMetrics
try:
    from steadypath_ros2.ros_compat import (
        HAS_ROS2, quaternion_to_yaw,
        MockOdometry, MockTwist, MockPath, MockString
    )
except ImportError:
    from steadypath_ros2.steadypath_ros2.ros_compat import (
        HAS_ROS2, quaternion_to_yaw,
        MockOdometry, MockTwist, MockPath, MockString
    )

if HAS_ROS2:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
else:
    Node = object
    Odometry = MockOdometry
    Twist = MockTwist
    Path = MockPath
    String = MockString


class TelemetryLoggerLogic:
    """
    Pure Python algorithmic logic bridging ROS 2 sensor/control streams
    into Snehal's 32-column Telemetry Logger.
    """

    def __init__(
        self,
        output_dir: Optional[str] = None,
        run_name: str = "ros2_live_session",
    ):
        log_dir = output_dir or os.path.join(PROJECT_ROOT, "analysis", "telemetry_logs")
        os.makedirs(log_dir, exist_ok=True)

        self.logger = SteadyPathLogger(
            output_dir=log_dir,
            run_id=run_name,
            buffer_size=10,
        )
        self.logger.start_run({"source": "ROS2_TELEMETRY_NODE"})

        self.step_idx = 0
        self.start_time = time.time()

        # State caches
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.curr_v = 0.0
        self.curr_omega = 0.0

        self.cmd_accel = 0.0
        self.cmd_steer = 0.0
        self.last_accel = 0.0
        self.last_steer = 0.0

        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_yaw = 0.0
        self.ref_v = 1.2
        self.ref_curvature = 0.0

        self.lat_error = 0.0
        self.head_error = 0.0
        self.vel_error = 0.0
        self.long_error = 0.0

        self.mpc_comp_time = 0.0
        self.mpc_status = 1
        self.pred_horizon: List[List[float]] = []

        self.replan_count = 0
        self.replan_in_progress = False
        self.active_destination_id = "DEST_BAY_01"

    def update_odom(self, x: float, y: float, yaw: float, v: float, omega: float):
        self.curr_x = x
        self.curr_y = y
        self.curr_yaw = yaw
        self.curr_v = v
        self.curr_omega = omega

    def update_control_status(self, status_dict: Dict[str, Any]):
        self.lat_error = float(status_dict.get("lateral_error", 0.0))
        self.head_error = float(status_dict.get("heading_error", 0.0))
        self.mpc_comp_time = float(status_dict.get("comp_time_ms", 0.0))
        self.cmd_accel = float(status_dict.get("accel_cmd", 0.0))
        self.cmd_steer = float(status_dict.get("steering_cmd_rad", 0.0))
        self.mpc_status = 1 if status_dict.get("status") == "OPTIMAL" else 0

    def update_replan_event(self, event_dict: Dict[str, Any]):
        self.replan_count = int(event_dict.get("replan_count", self.replan_count + 1))
        self.replan_in_progress = True

    def record_step(self) -> TelemetryRecord:
        """Capture record and write to CSV & JSONL."""
        now_ts = round(time.time() - self.start_time, 4)

        jerk_accel = (self.cmd_accel - self.last_accel) / 0.02 if self.step_idx > 0 else 0.0
        steer_rate = (self.cmd_steer - self.last_steer) / 0.02 if self.step_idx > 0 else 0.0
        self.last_accel = self.cmd_accel
        self.last_steer = self.cmd_steer

        rec = TelemetryRecord(
            timestamp=now_ts,
            step=self.step_idx,
            x=round(self.curr_x, 4),
            y=round(self.curr_y, 4),
            yaw=round(self.curr_yaw, 4),
            v=round(self.curr_v, 4),
            omega=round(self.curr_omega, 4),
            x_ref=round(self.ref_x, 4),
            y_ref=round(self.ref_y, 4),
            yaw_ref=round(self.ref_yaw, 4),
            v_ref=round(self.ref_v, 4),
            curvature_ref=round(self.ref_curvature, 4),
            lateral_error=round(self.lat_error, 5),
            heading_error=round(self.head_error, 5),
            velocity_error=round(self.curr_v - self.ref_v, 4),
            longitudinal_error=round(self.long_error, 4),
            accel=round(self.cmd_accel, 4),
            steering_angle=round(self.cmd_steer, 4),
            jerk_accel=round(jerk_accel, 4),
            steering_rate=round(steer_rate, 4),
            mpc_comp_time_ms=round(self.mpc_comp_time, 3),
            mpc_status=self.mpc_status,
            predicted_trajectory=self.pred_horizon,
            active_destination_id=self.active_destination_id,
            replan_count=self.replan_count,
            replan_in_progress=self.replan_in_progress,
        )

        self.logger.log_step(rec)
        self.step_idx += 1
        return rec

    def finalize(self) -> Dict[str, Any]:
        """Close log streams and compute summary metrics."""
        self.logger.end_run(final_status="SUCCESS")
        csv_path = self.logger.csv_path
        if os.path.exists(csv_path) and os.path.getsize(csv_path) > 100:
            data = SteadyPathLogger.load_csv(csv_path)
            evaluator = SteadyPathEvaluator()
            metrics = evaluator.evaluate(data, run_id=self.logger.run_id, controller_type="MPC")
            return metrics.to_dict()
        return {}


class SteadyPathTelemetryNode(Node):
    """
    ROS 2 Node for streaming telemetry logging and evaluation.
    """

    def __init__(self):
        super().__init__("steadypath_telemetry_logger_node")

        self.declare_parameter("log_frequency", 50.0)
        self.declare_parameter("run_name", "ros2_deployment_run")

        log_freq = self.get_parameter("log_frequency").value
        run_name = self.get_parameter("run_name").value

        self.logic = TelemetryLoggerLogic(run_name=run_name)

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.status_sub = self.create_subscription(String, "/steadypath/control_status", self.status_callback, 10)
        self.replan_sub = self.create_subscription(String, "/steadypath/replan_event", self.replan_callback, 10)
        self.path_sub = self.create_subscription(Path, "/global_plan", self.path_callback, 10)

        # Periodic logging timer
        self.timer = self.create_timer(1.0 / log_freq, self.log_cycle)

        self.get_logger().info(
            f"SteadyPath Telemetry Logger Node Active. Output CSV: {self.logic.logger.csv_path}"
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        self.logic.update_odom(x, y, yaw, v, omega)

    def status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.logic.update_control_status(data)
        except Exception:
            pass

    def replan_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.logic.update_replan_event(data)
        except Exception:
            pass

    def path_callback(self, msg: Path):
        if msg.poses:
            # Set active destination from end of path
            last_p = msg.poses[-1].pose.position
            self.logic.ref_x = last_p.x
            self.logic.ref_y = last_p.y

    def log_cycle(self):
        self.logic.record_step()

    def destroy_node(self):
        self.get_logger().info("Finalizing Telemetry Logs & Computing Run Metrics...")
        metrics = self.logic.finalize()
        if metrics:
            self.get_logger().info(
                f"[METRICS] Mean Lat Error: {metrics.get('mean_lateral_error', 0.0):.4f} m | "
                f"Max Lat Error: {metrics.get('max_lateral_error', 0.0):.4f} m | "
                f"Mean Comp Time: {metrics.get('mean_mpc_comp_time_ms', 0.0):.2f} ms"
            )
        super().destroy_node()


def main(args=None):
    if not HAS_ROS2:
        print("[ERROR] rclpy is not installed on this system. Please run in a ROS 2 container.")
        sys.exit(1)
    rclpy.init(args=args)
    node = SteadyPathTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
