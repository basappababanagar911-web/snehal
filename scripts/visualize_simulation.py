"""
visualize_simulation.py
=======================
Live Interactive Desktop Graphical Visualizer for SteadyPath AGV System.

Opens a 50 Hz real-time animated simulation window on your desktop screen:
- 2D Warehouse floor with aisles, storage racks, and loading bays.
- AGV kinematic bicycle model with animated chassis and front steering.
- Dynamic obstacle avoidance (moving crossing forklift or corridor blockage).
- Live telemetry strip charts (lateral error, heading error, steering, speed).

Usage:
    .\.venv\Scripts\python.exe visualize_simulation.py
    .\.venv\Scripts\python.exe visualize_simulation.py --controller mpc --scenario replan
    .\.venv\Scripts\python.exe visualize_simulation.py --controller stanley --scenario curved
"""

import sys
import os
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from planner.route_planner import RoutePlanner
from simulation.warehouse_env import WarehouseEnvironment
from mpc.mpc_controller import SteadyPathMPC
from mpc.stanley_controller import StanleyController


class SteadyPathVisualizer:
    def __init__(self, controller_name="mpc", scenario="replan", destination_id="DEST_BAY_01", speed_multiplier=1):
        self.controller_name = controller_name.lower()
        self.scenario = scenario.lower()
        self.destination_id = destination_id
        self.speed_multiplier = speed_multiplier
        self.dt = 0.05

        # Initialize planner
        self.planner = RoutePlanner(default_velocity=1.5)
        self.route = self.planner.plan(destination_id=self.destination_id)
        self.replan_triggered = False

        # Initialize simulation physics
        self.env = WarehouseEnvironment(dt=self.dt, wheelbase=1.2)
        initial_yaw = self.route.waypoints[0].yaw if self.route.waypoints else 0.465
        self.env.reset(np.array([0.0, 0.0, initial_yaw, 0.0]))

        # Configure obstacles
        self.blockage_scheduled = (self.scenario == "replan")
        self.moving_obstacle_active = (self.scenario == "moving")

        if self.moving_obstacle_active:
            self.env.add_moving_obstacle(start_x=14.0, start_y=-3.0, vx=0.0, vy=0.30, radius=0.55, obs_id="crossing_forklift")

        # Initialize controller
        if self.controller_name == "mpc":
            self.controller = SteadyPathMPC(wheelbase=1.2, dt=self.dt, horizon=15)
        else:
            self.controller = StanleyController(wheelbase=1.2, k_e=0.95)

        # Telemetry history buffers for plotting
        self.history_time = []
        self.history_lat_err = []
        self.history_steer = []
        self.history_speed = []
        self.history_trail_x = []
        self.history_trail_y = []

        # Target goal coordinates
        self.dest_coords = RoutePlanner.DESTINATIONS.get(self.destination_id, (30.0, 1.902))

        # Setup Matplotlib Figure
        self.setup_ui()

    def setup_ui(self):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 8), dpi=100)
        self.fig.canvas.manager.set_window_title(f"SteadyPath AGV Live Simulation [{self.controller_name.upper()} - {self.scenario.upper()}]")

        # Grid: Top 60% for warehouse map, bottom 40% for telemetry charts
        gs = self.fig.add_gridspec(2, 3, height_ratios=[1.6, 1.0], hspace=0.32, wspace=0.25)

        # 1. Main Warehouse Map Viewport
        self.ax_map = self.fig.add_subplot(gs[0, :])
        self.ax_map.set_xlim(-2, 42)
        self.ax_map.set_ylim(-5.5, 5.5)
        self.ax_map.set_aspect('equal')
        self.ax_map.set_title(f"SteadyPath Autonomous Guided Vehicle (AGV) Warehouse Navigation - [{self.controller_name.upper()}]", fontsize=12, fontweight='bold', color='#38bdf8', pad=10)
        self.ax_map.set_xlabel("Warehouse Longitudinal Axis X (meters)", fontsize=10)
        self.ax_map.set_ylabel("Lateral Axis Y (meters)", fontsize=10)
        self.ax_map.grid(True, linestyle=':', alpha=0.3, color='#475569')

        # Draw static warehouse storage racks
        for rx in range(2, 38, 7):
            # Top rack
            self.ax_map.add_patch(patches.Rectangle((rx, 2.8), 5.0, 2.2, facecolor='#1e293b', edgecolor='#475569', linewidth=1.5))
            self.ax_map.text(rx + 2.5, 3.9, "RACK", color='#64748b', fontsize=8, ha='center', va='center')
            # Bottom rack
            self.ax_map.add_patch(patches.Rectangle((rx, -5.0), 5.0, 2.2, facecolor='#1e293b', edgecolor='#475569', linewidth=1.5))
            self.ax_map.text(rx + 2.5, -3.9, "RACK", color='#64748b', fontsize=8, ha='center', va='center')

        # Draw warehouse destinations
        for bay_id, (bx, by) in RoutePlanner.DESTINATIONS.items():
            is_active = (bay_id == self.destination_id)
            color = '#38bdf8' if is_active else '#64748b'
            self.ax_map.plot(bx, by, marker='*', markersize=14 if is_active else 8, color=color)
            self.ax_map.text(bx, by + 0.55, bay_id.replace("DEST_", ""), color=color, fontsize=9, fontweight='bold' if is_active else 'normal', ha='center')

        # Plot Reference Path
        wps_x = [wp.x for wp in self.route.waypoints]
        wps_y = [wp.y for wp in self.route.waypoints]
        self.line_ref, = self.ax_map.plot(wps_x, wps_y, color='#facc15', linestyle='--', linewidth=2, label="Planned Route", zorder=2)
        self.line_trail, = self.ax_map.plot([], [], color='#3b82f6', linewidth=2.5, label="AGV Actual Path", zorder=3)
        self.line_horizon, = self.ax_map.plot([], [], color='#10b981', linestyle=':', marker='o', markersize=3, linewidth=1.8, label="MPC Predicted Horizon", zorder=4)

        # Obstacle patches
        self.obstacle_patches = []

        # AGV Vehicle Chassis representation
        self.agv_patch = patches.Rectangle((0, 0), 1.6, 0.85, angle=0.0, facecolor='#f97316', edgecolor='#ffffff', linewidth=1.8, zorder=5)
        self.ax_map.add_patch(self.agv_patch)
        self.steer_line, = self.ax_map.plot([], [], color='#fde047', linewidth=3, zorder=6)

        # Status annotation overlay
        self.txt_status = self.ax_map.text(0.02, 0.92, "", transform=self.ax_map.transAxes, fontsize=10, fontweight='bold', color='#22c55e',
                                          bbox=dict(boxstyle='round,pad=0.5', facecolor='#0f172a', edgecolor='#334155', alpha=0.9))
        self.ax_map.legend(loc='upper right', fontsize=8, facecolor='#0f172a', edgecolor='#334155')

        # 2. Subplot: Lateral Error Chart
        self.ax_lat = self.fig.add_subplot(gs[1, 0])
        self.ax_lat.set_title("Lateral Deviation $e_{lat}$ (cm)", fontsize=10, color='#38bdf8')
        self.ax_lat.set_xlabel("Time (s)", fontsize=8)
        self.ax_lat.set_ylabel("Error (cm)", fontsize=8)
        self.ax_lat.axhline(0, color='#64748b', linestyle='--', linewidth=0.8)
        self.ax_lat.axhline(5, color='#ef4444', linestyle=':', alpha=0.6, label="5cm Bound")
        self.ax_lat.axhline(-5, color='#ef4444', linestyle=':', alpha=0.6)
        self.line_lat_err, = self.ax_lat.plot([], [], color='#38bdf8', linewidth=1.8)
        self.ax_lat.grid(True, linestyle=':', alpha=0.3)
        self.ax_lat.set_ylim(-15, 15)

        # 3. Subplot: Steering Angle
        self.ax_steer = self.fig.add_subplot(gs[1, 1])
        self.ax_steer.set_title("Steering Command $\\delta$ (deg)", fontsize=10, color='#facc15')
        self.ax_steer.set_xlabel("Time (s)", fontsize=8)
        self.ax_steer.set_ylabel("Steer Angle (°)", fontsize=8)
        self.line_steer_err, = self.ax_steer.plot([], [], color='#facc15', linewidth=1.8)
        self.ax_steer.grid(True, linestyle=':', alpha=0.3)
        self.ax_steer.set_ylim(-35, 35)

        # 4. Subplot: Live Telemetry Metrics Card
        self.ax_card = self.fig.add_subplot(gs[1, 2])
        self.ax_card.axis('off')
        self.txt_telemetry = self.ax_card.text(
            0.05, 0.95, "", transform=self.ax_card.transAxes, fontsize=10, fontfamily='monospace',
            verticalalignment='top',
            bbox=dict(boxstyle='round,pad=0.8', facecolor='#0f172a', edgecolor='#38bdf8', alpha=0.95)
        )

    def step(self):
        curr_state = self.env.state.copy()
        curr_time = self.env.time
        veh_x, veh_y, veh_yaw, veh_v = curr_state

        # Check goal distance
        gx, gy = self.dest_coords
        dist_to_goal = math.hypot(veh_x - gx, veh_y - gy)
        if dist_to_goal < 0.12 and curr_time > 2.0:
            return False

        # Schedule static blockage in replan scenario
        if self.blockage_scheduled and curr_time >= 4.0 and not self.replan_triggered:
            self.env.add_static_obstacle(x=14.0, y=0.5, radius=0.6, obs_id="blocked_pallet")
            # Trigger dynamic replanner
            self.route = self.planner.plan(destination_id=self.destination_id, replan_blockage=(14.0, 0.5))
            self.replan_triggered = True
            # Update path display
            wps_x = [wp.x for wp in self.route.waypoints]
            wps_y = [wp.y for wp in self.route.waypoints]
            self.line_ref.set_data(wps_x, wps_y)
            self.line_ref.set_color('#38bdf8')

        # Find closest reference waypoint
        best_idx = 0
        min_d = float('inf')
        for i, wp in enumerate(self.route.waypoints):
            d = (wp.x - veh_x)**2 + (wp.y - veh_y)**2
            if d < min_d:
                min_d = d
                best_idx = i

        ref_wp = self.route.waypoints[best_idx]

        # Calculate Frenet tracking errors
        dx = veh_x - ref_wp.x
        dy = veh_y - ref_wp.y
        lat_error = -math.sin(ref_wp.yaw) * dx + math.cos(ref_wp.yaw) * dy
        heading_error = math.atan2(math.sin(veh_yaw - ref_wp.yaw), math.cos(veh_yaw - ref_wp.yaw))

        # Solve control
        horizon_coords = []
        if self.controller_name == "mpc":
            ref_window = self.planner.get_reference_window(veh_x, horizon_steps=15, dt=self.dt)
            accel, steer, comp_time, status, pred_traj = self.controller.solve(curr_state, ref_window)
            horizon_coords = [(p[0], p[1]) for p in pred_traj]
        else:
            accel, steer = self.controller.compute_control(curr_state, ref_wp.x, ref_wp.y, ref_wp.yaw, ref_wp.v)
            comp_time = 0.2

        # Step simulation environment
        snapshot = self.env.step(accel=accel, steer=steer)

        # Log history
        self.history_time.append(curr_time)
        self.history_lat_err.append(lat_error * 100.0)  # cm
        self.history_steer.append(math.degrees(steer))
        self.history_speed.append(veh_v)
        self.history_trail_x.append(veh_x)
        self.history_trail_y.append(veh_y)

        # Update vehicle visualization
        # Transform AGV rectangle: anchor at rear-center
        rad = veh_yaw
        cos_y = math.cos(rad)
        sin_y = math.sin(rad)
        # Offset to center the 1.6m x 0.85m box
        cx = veh_x - 0.3 * cos_y + 0.425 * sin_y
        cy = veh_y - 0.3 * sin_y - 0.425 * cos_y
        self.agv_patch.set_xy((cx, cy))
        self.agv_patch.angle = math.degrees(rad)

        # Steering indicator
        front_x = veh_x + 1.2 * cos_y
        front_y = veh_y + 1.2 * sin_y
        steer_len = 0.6
        steer_angle = rad + steer
        self.steer_line.set_data([front_x, front_x + steer_len * math.cos(steer_angle)],
                                 [front_y, front_y + steer_len * math.sin(steer_angle)])

        # Trail
        self.line_trail.set_data(self.history_trail_x, self.history_trail_y)

        # Horizon
        if horizon_coords:
            hx = [p[0] for p in horizon_coords]
            hy = [p[1] for p in horizon_coords]
            self.line_horizon.set_data(hx, hy)

        # Obstacles
        for p in self.obstacle_patches:
            p.remove()
        self.obstacle_patches = []

        for obs in self.env.obstacles:
            color = '#ef4444' if not obs.is_moving else '#f97316'
            patch = patches.Circle((obs.x, obs.y), obs.radius, facecolor=color, edgecolor='#ffffff', linewidth=1.5, zorder=4)
            self.ax_map.add_patch(patch)
            self.obstacle_patches.append(patch)
            # Safety margin
            halo = patches.Circle((obs.x, obs.y), obs.radius + 0.5, facecolor='none', edgecolor=color, linestyle='--', linewidth=1.0, alpha=0.6, zorder=3)
            self.ax_map.add_patch(halo)
            self.obstacle_patches.append(halo)

        # Status text
        if self.replan_triggered:
            status_str = f"⚡ DYNAMIC REPLAN ACTIVE (Avoiding Obstacle) | Clearance: {snapshot.min_clearance:.2f}m"
            status_col = '#38bdf8'
        elif snapshot.min_clearance < 0.8:
            status_str = f"⚠️ OBSTACLE NEARBY | Clearance: {snapshot.min_clearance:.2f}m"
            status_col = '#f59e0b'
        else:
            status_str = f"✓ NOMINAL CRUISE | Clearance: {snapshot.min_clearance:.2f}m"
            status_col = '#22c55e'

        self.txt_status.set_text(status_str)
        self.txt_status.set_color(status_col)

        # Update telemetry strip charts
        t_recent = self.history_time[-120:]
        lat_recent = self.history_lat_err[-120:]
        steer_recent = self.history_steer[-120:]

        self.line_lat_err.set_data(t_recent, lat_recent)
        if len(t_recent) > 1:
            self.ax_lat.set_xlim(t_recent[0], max(t_recent[-1], t_recent[0] + 5.0))

        self.line_steer_err.set_data(t_recent, steer_recent)
        if len(t_recent) > 1:
            self.ax_steer.set_xlim(t_recent[0], max(t_recent[-1], t_recent[0] + 5.0))

        # Telemetry Card
        mean_lat = np.mean(np.abs(self.history_lat_err)) if self.history_lat_err else 0.0
        max_lat = np.max(np.abs(self.history_lat_err)) if self.history_lat_err else 0.0

        card_text = (
            f"TELEMETRY STREAM:\n"
            f"---------------------------\n"
            f"TIME        : {curr_time:6.2f} s\n"
            f"VELOCITY    : {veh_v:6.2f} m/s\n"
            f"STEER ANGLE : {math.degrees(steer):6.2f} deg\n"
            f"LAT ERROR   : {lat_error * 100:6.2f} cm\n"
            f"MEAN ERROR  : {mean_lat:6.2f} cm\n"
            f"PEAK ERROR  : {max_lat:6.2f} cm\n"
            f"CLEARANCE   : {snapshot.min_clearance:6.2f} m\n"
            f"SOLVER TIME : {comp_time:6.2f} ms\n"
            f"DESTINATION : {self.destination_id}\n"
            f"DIST TO GOAL: {dist_to_goal:6.2f} m"
        )
        self.txt_telemetry.set_text(card_text)

        return True

    def run(self):
        def update_frame(frame):
            for _ in range(self.speed_multiplier):
                active = self.step()
                if not active:
                    self.txt_status.set_text("★ MISSION COMPLETE: TARGET DESTINATION REACHED ★")
                    self.txt_status.set_color('#facc15')
                    break
            return self.agv_patch, self.line_trail, self.line_horizon, self.line_lat_err, self.line_steer_err

        anim = FuncAnimation(self.fig, update_frame, interval=40, blit=False, cache_frame_data=False)
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="SteadyPath Live Simulation Visualizer")
    parser.add_argument("--controller", type=str, default="mpc", choices=["mpc", "stanley"], help="Controller to run")
    parser.add_argument("--scenario", type=str, default="replan", choices=["normal", "replan", "moving"], help="Scenario to simulate")
    parser.add_argument("--destination", type=str, default="DEST_BAY_01", help="Target bay destination")
    parser.add_argument("--speed", type=int, default=1, choices=[1, 2, 4], help="Simulation speed multiplier")
    args = parser.parse_args()

    vis = SteadyPathVisualizer(
        controller_name=args.controller,
        scenario=args.scenario,
        destination_id=args.destination,
        speed_multiplier=args.speed,
    )
    vis.run()


if __name__ == "__main__":
    main()
