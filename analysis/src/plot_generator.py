"""
plot_generator.py
=================
SteadyPath Publication-Quality Visualization Engine (Snehal's Module - Day 12)

Generates clean, academic-grade plots for report, paper, and presentation:
1. Trajectory Tracking Map (2D Warehouse XY with obstacles, start, goal)
2. Tracking Errors vs. Time (Lateral error, Heading error, Velocity error)
3. Control Signals & MPC Latency (Acceleration, Steering angle, Solver time)
4. Obstacle Clearance vs. Time (with 0.5m safety boundary and collision zone)
5. Dynamic Replanning Visualization (Original path, blockage, reroute)
6. Baseline Comparison (SteadyPath MPC vs. Stanley controller)
"""

import os
import math
from typing import Dict, List, Optional, Tuple, Any
import matplotlib
matplotlib.use("Agg")  # Non-interactive headless backend for automated pipelines
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


# Academic styling parameters
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 10,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "figure.titlesize": 13,
    "lines.linewidth": 1.75,
    "grid.alpha": 0.35,
    "grid.linestyle": "--"
})


class SteadyPathPlotter:
    """Creates publication-ready visualizations from logged telemetry data."""

    def __init__(self, output_dir: str = "analysis/graphs"):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

    def plot_trajectory_2d(
        self,
        data: Dict[str, List[Any]],
        obstacles: Optional[List[Dict[str, float]]] = None,
        title: str = "SteadyPath Vehicle Trajectory Tracking",
        filename: str = "trajectory_tracking.png"
    ) -> str:
        """Plots 2D XY vehicle trajectory against reference trajectory and obstacles."""
        fig, ax = plt.subplots(figsize=(8, 6), dpi=300)

        # Reference trajectory
        ax.plot(data["x_ref"], data["y_ref"], "k--", label="Reference Trajectory", alpha=0.7)
        # Actual trajectory
        ax.plot(data["x"], data["y"], "b-", label=f"Actual Path ({data.get('controller_type', 'MPC')})")

        # Start and Goal markers
        ax.plot(data["x"][0], data["y"][0], "go", markersize=8, label="Start $(x_0, y_0)$")
        ax.plot(data["goal_x"][-1], data["goal_y"][-1], "r*", markersize=12, label="Goal Destination")

        # Plot obstacles if provided
        if obstacles:
            for obs in obstacles:
                circle = plt.Circle(
                    (obs["x"], obs["y"]),
                    obs.get("radius", 0.5),
                    color="crimson",
                    alpha=0.6,
                    label="Obstacle" if "Obstacle" not in [l.get_label() for l in ax.lines] else ""
                )
                ax.add_patch(circle)

        ax.set_xlabel("Warehouse X [m]")
        ax.set_ylabel("Warehouse Y [m]")
        ax.set_title(title, pad=12, fontweight="bold")
        ax.axis("equal")
        ax.grid(True)
        ax.legend(loc="best", framealpha=0.9)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        return out_path

    def plot_tracking_errors(
        self,
        data: Dict[str, List[Any]],
        filename: str = "tracking_errors.png"
    ) -> str:
        """Plots 3-panel synchronized lateral, heading, and velocity errors."""
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 7), sharex=True, dpi=300)
        t = data["timestamp"]

        # Lateral Error
        ax1.plot(t, data["lateral_error"], color="royalblue", label="Lateral Error $e_{lat}$")
        ax1.axhline(0, color="black", linestyle="--", linewidth=0.8)
        ax1.axhline(0.15, color="red", linestyle=":", label="Tolerance Limit (±0.15m)")
        ax1.axhline(-0.15, color="red", linestyle=":")
        ax1.set_ylabel("Lat Error [m]")
        ax1.set_title("SteadyPath Tracking Errors vs. Time", fontweight="bold")
        ax1.grid(True)
        ax1.legend(loc="upper right")

        # Heading Error (degrees)
        heading_err_deg = [math.degrees(e) for e in data["heading_error"]]
        ax2.plot(t, heading_err_deg, color="forestgreen", label="Heading Error $e_{\psi}$")
        ax2.axhline(0, color="black", linestyle="--", linewidth=0.8)
        ax2.axhline(5.0, color="red", linestyle=":", label="Tolerance Limit (±5°)")
        ax2.axhline(-5.0, color="red", linestyle=":")
        ax2.set_ylabel("Yaw Error [deg]")
        ax2.grid(True)
        ax2.legend(loc="upper right")

        # Velocity Tracking
        ax3.plot(t, data["v_ref"], "k--", label="Reference Speed $v_{ref}$", alpha=0.7)
        ax3.plot(t, data["v"], color="darkorange", label="Actual Speed $v$")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Speed [m/s]")
        ax3.grid(True)
        ax3.legend(loc="upper right")

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        return out_path

    def plot_control_signals(
        self,
        data: Dict[str, List[Any]],
        filename: str = "control_signals.png"
    ) -> str:
        """Plots acceleration, steering angle, and MPC computation latency."""
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 7), sharex=True, dpi=300)
        t = data["timestamp"]

        # Acceleration
        ax1.plot(t, data["accel"], color="purple", label="Applied Acceleration $a$")
        ax1.set_ylabel("Accel [$m/s^2$]")
        ax1.set_title("Controller Behavior & Computational Latency", fontweight="bold")
        ax1.grid(True)
        ax1.legend(loc="upper right")

        # Steering Angle (degrees)
        steering_deg = [math.degrees(s) for s in data["steering_angle"]]
        ax2.plot(t, steering_deg, color="teal", label="Front Steering Angle $\\delta$")
        ax2.set_ylabel("Steering [deg]")
        ax2.grid(True)
        ax2.legend(loc="upper right")

        # MPC Computation Time
        ax3.plot(t, data["mpc_comp_time_ms"], color="crimson", label="MPC Latency $\\tau_{comp}$")
        ax3.axhline(20.0, color="orange", linestyle="--", label="Budget Target (20 ms)")
        ax3.axhline(50.0, color="red", linestyle=":", label="Real-time Cutoff $\\Delta t$ (50 ms)")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Latency [ms]")
        ax3.grid(True)
        ax3.legend(loc="upper right")

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        return out_path

    def plot_obstacle_clearance(
        self,
        data: Dict[str, List[Any]],
        safety_threshold: float = 0.50,
        filename: str = "obstacle_clearance.png"
    ) -> str:
        """Plots clearance to nearest obstacle over time with safety zones."""
        fig, ax = plt.subplots(figsize=(8, 4.5), dpi=300)
        t = data["timestamp"]
        clearance = data["min_clearance"]

        ax.plot(t, clearance, color="navy", label="Nearest Obstacle Clearance $d_{min}$")
        ax.axhline(safety_threshold, color="red", linestyle="--", linewidth=1.5, label=f"Safety Margin ({safety_threshold:.1f} m)")
        
        # Shade violation area
        ax.axhspan(0, safety_threshold, color="red", alpha=0.15, label="Safety Buffer Breach")

        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Clearance Distance [m]")
        ax.set_title("Dynamic Obstacle Clearance Profile", pad=12, fontweight="bold")
        ax.set_ylim(bottom=0.0, top=max(min(max(clearance), 5.0), 2.0))
        ax.grid(True)
        ax.legend(loc="upper right", framealpha=0.9)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        return out_path

    def plot_dynamic_replan_event(
        self,
        nominal_path: Tuple[List[float], List[float]],
        replan_path: Tuple[List[float], List[float]],
        actual_path: Tuple[List[float], List[float]],
        blockage_pos: Tuple[float, float],
        replan_trigger_pos: Tuple[float, float],
        goal_pos: Tuple[float, float],
        filename: str = "dynamic_replan_event.png"
    ) -> str:
        """Snehal Proof 2: Visual proof of dynamic route replanning under blockage."""
        fig, ax = plt.subplots(figsize=(9, 6.5), dpi=300)

        # 1. Original nominal path (blocked)
        ax.plot(nominal_path[0], nominal_path[1], "k--", alpha=0.5, label="Original Nominal Route $R_1$")

        # 2. Replanned alternate path
        ax.plot(replan_path[0], replan_path[1], "g--", linewidth=2.0, label="Dynamic Alternate Route $R_2$")

        # 3. Actual vehicle path taken
        ax.plot(actual_path[0], actual_path[1], "b-", linewidth=2.5, label="Executed Vehicle Path")

        # Blockage marker
        block_circle = plt.Circle(blockage_pos, 0.8, color="red", alpha=0.8, label="Detected Corridor Blockage")
        ax.add_patch(block_circle)

        # Replan trigger point
        ax.plot(replan_trigger_pos[0], replan_trigger_pos[1], "y^", markersize=11, label="Replan Trigger Point")

        # Goal
        ax.plot(goal_pos[0], goal_pos[1], "r*", markersize=14, label="Destination (Goal)")

        ax.set_xlabel("Warehouse X [m]")
        ax.set_ylabel("Warehouse Y [m]")
        ax.set_title("SteadyPath Dynamic Replanning Under Corridor Blockage", pad=12, fontweight="bold")
        ax.axis("equal")
        ax.grid(True)
        ax.legend(loc="best", framealpha=0.9)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        return out_path

    def plot_stanley_vs_mpc_comparison(
        self,
        stanley_data: Dict[str, List[Any]],
        mpc_data: Dict[str, List[Any]],
        filename: str = "stanley_vs_mpc_benchmark.png"
    ) -> str:
        """Snehal Proof 3: Head-to-head trajectory and error comparison."""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(11, 8), dpi=300)

        # Top-left: Trajectories
        ax1.plot(mpc_data["x_ref"], mpc_data["y_ref"], "k--", alpha=0.6, label="Reference")
        ax1.plot(stanley_data["x"], stanley_data["y"], "r-", label="Stanley Baseline")
        ax1.plot(mpc_data["x"], mpc_data["y"], "b-", label="SteadyPath MPC")
        ax1.set_xlabel("X [m]")
        ax1.set_ylabel("Y [m]")
        ax1.set_title("Trajectory Comparison", fontweight="bold")
        ax1.axis("equal")
        ax1.grid(True)
        ax1.legend()

        # Top-right: Lateral Error vs Time
        ax2.plot(stanley_data["timestamp"], stanley_data["lateral_error"], "r-", alpha=0.8, label="Stanley")
        ax2.plot(mpc_data["timestamp"], mpc_data["lateral_error"], "b-", label="SteadyPath MPC")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Lateral Error [m]")
        ax2.set_title("Lateral Error vs. Time", fontweight="bold")
        ax2.grid(True)
        ax2.legend()

        # Bottom-left: Steering Angle vs Time
        s_steer_deg = [math.degrees(s) for s in stanley_data["steering_angle"]]
        m_steer_deg = [math.degrees(s) for s in mpc_data["steering_angle"]]
        ax3.plot(stanley_data["timestamp"], s_steer_deg, "r-", alpha=0.8, label="Stanley")
        ax3.plot(mpc_data["timestamp"], m_steer_deg, "b-", label="SteadyPath MPC")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Steering Angle [deg]")
        ax3.set_title("Steering Smoothness Comparison", fontweight="bold")
        ax3.grid(True)
        ax3.legend()

        # Bottom-right: Error Distribution Boxplot
        box_data = [
            [abs(e) for e in stanley_data["lateral_error"]],
            [abs(e) for e in mpc_data["lateral_error"]]
        ]
        ax4.boxplot(box_data, labels=["Stanley", "SteadyPath MPC"], patch_artist=True,
                    boxprops=dict(facecolor="lightblue", color="blue"),
                    medianprops=dict(color="red", linewidth=1.5))
        ax4.set_ylabel("Absolute Lateral Error [m]")
        ax4.set_title("Lateral Error Distribution", fontweight="bold")
        ax4.grid(True)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        return out_path
