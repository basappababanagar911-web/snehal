"""
integrate.py
============
SteadyPath Master Integration Engine (YOU - MPC & Integration Lead)

Connects the four core team modules into an autonomous closed loop:
1. R.P. Singh: Global route planning & dynamic blockage replanning (planner/)
2. YOU: Real-time Model Predictive Controller / Stanley baseline (mpc/)
3. B. Sheshank: 3D warehouse simulation physics & obstacle tracking (simulation/)
4. Snehal: Telemetry recording, analytics, and metrics evaluation (analysis/)

Usage:
    python integrate.py --controller mpc --scenario normal
    python integrate.py --controller stanley --scenario curved
    python integrate.py --controller mpc --scenario replan
    python integrate.py --controller mpc --scenario moving
    python integrate.py --controller mpc --scenario all
"""

import os
import sys
import math
import argparse
import numpy as np

# Add project root and analysis src to path
root_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, root_dir)
sys.path.insert(0, os.path.join(root_dir, "analysis", "src"))

from planner.route_planner import RoutePlanner
from simulation.warehouse_env import WarehouseEnvironment
from mpc.mpc_controller import SteadyPathMPC
from mpc.stanley_controller import StanleyController
from analysis.src.data_logger import SteadyPathLogger, TelemetryRecord
from analysis.src.compute_metrics import MetricsBatchProcessor


def run_integrated_simulation(
    run_id: str,
    controller_name: str = "MPC",
    destination_id: str = "DEST_BAY_01",
    scenario: str = "normal",
    max_duration: float = 35.0,
    dt: float = 0.05
):
    """Executes an end-to-end closed-loop integrated simulation."""
    print(f"\n========================================================")
    print(f"  SteadyPath Integration: {run_id} [{controller_name}]")
    print(f"  Destination: {destination_id} | Scenario: {scenario}")
    print(f"========================================================")

    # 1. Initialize R.P. Singh's Planner
    planner = RoutePlanner(default_velocity=1.5)
    route = planner.plan(destination_id=destination_id)
    print(f"[Planner] Generated nominal route: {route.total_length:.2f}m to {destination_id}")

    # 2. Initialize B. Sheshank's Warehouse Simulation
    env = WarehouseEnvironment(dt=dt, wheelbase=1.2)
    initial_yaw = route.waypoints[0].yaw if route.waypoints else 0.0
    env.reset(initial_state=np.array([0.0, 0.0, initial_yaw, 0.0]))

    # Setup scenario obstacles
    blockage_coord = None
    if scenario in ("static", "stress"):
        env.add_static_obstacle(x=8.0, y=2.8, radius=0.4, obs_id="rack_pillar_A")
        env.add_static_obstacle(x=15.0, y=-2.7, radius=0.45, obs_id="rack_pillar_B")
        env.add_static_obstacle(x=22.0, y=2.9, radius=0.4, obs_id="rack_pillar_C")
        print("[Sim] Spawned static storage rack pillars.")

    if scenario == "moving":
        env.add_moving_obstacle(start_x=16.0, start_y=-3.5, vx=0.0, vy=0.35, radius=0.45, obs_id="crossing_forklift")
        print("[Sim] Spawned crossing moving forklift at x=16.0m.")

    if scenario == "replan":
        blockage_coord = (14.0, 0.0)
        # Blockage appears at t=5.0s
        print("[Sim] Scheduled corridor blockage at x=14.0m (t=5.0s).")

    # 3. Initialize Controller (YOU)
    if controller_name.upper() == "MPC":
        controller = SteadyPathMPC(wheelbase=1.2, dt=dt, horizon=15)
        print("[MPC] SteadyPath Non-Linear MPC Initialized (Horizon N=15).")
    else:
        controller = StanleyController(wheelbase=1.2, dt=dt)
        print("[Stanley] Baseline Stanley Geometric Controller Initialized.")

    # 4. Initialize Snehal's Data Logger
    raw_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analysis", "raw_data")
    logger = SteadyPathLogger(
        run_id=run_id,
        controller_type=controller_name,
        output_dir=raw_dir,
        export_csv=True,
        export_jsonl=True
    )
    logger.start_run(metadata={
        "run_id": run_id,
        "controller": controller_name,
        "destination": destination_id,
        "scenario": scenario,
        "route_length": route.total_length
    })

    # Closed-Loop Control Loop
    step = 0
    t = 0.0
    goal_coord = route.goal_coord
    replan_executed = False
    replan_count = 0

    applied_accel = 0.0
    applied_steer = 0.0

    while t <= max_duration:
        # Step Sheshank's physics
        sensor = env.step(accel=applied_accel, steer=applied_steer)
        veh_state = env.state  # [x, y, psi, v]

        dist_to_goal = math.hypot(veh_state[0] - goal_coord[0], veh_state[1] - goal_coord[1])
        if dist_to_goal < 0.08 and t > 3.0:
            print(f"[Mission] Target destination reached at t={t:.2f}s! Final Error: {dist_to_goal:.4f}m")
            break
        if veh_state[0] >= goal_coord[0]:
            break

        # Spawn scheduled blockage
        if scenario == "replan" and t >= 5.0 and blockage_coord is not None and not replan_executed:
            env.add_static_obstacle(x=blockage_coord[0], y=blockage_coord[1], radius=0.5, obs_id="corridor_blockage")
            logger.log_event("CORRIDOR_BLOCKED", {"pos": list(blockage_coord)}, timestamp=t)
            
            # Trigger R.P. Singh's Dynamic Replanner
            route = planner.plan(destination_id=destination_id, replan_blockage=blockage_coord)
            goal_coord = route.goal_coord
            replan_executed = True
            replan_count = route.replan_count
            logger.log_event("REPLAN_TRIGGERED", {"new_route_id": route.route_id, "detour_len": route.total_length}, timestamp=t)
            print(f"[Planner] Alert: Blockage at x={blockage_coord[0]}m -> Dynamic Replanning Triggered!")

        # Dynamic speed regulation if moving obstacle approaches
        nominal_speed = 1.5
        for m_obs in sensor.moving_obstacles:
            d_mov = math.hypot(veh_state[0] - m_obs["x"], veh_state[1] - m_obs["y"])
            if d_mov < 2.5:
                nominal_speed = 0.45  # Yield to crossing vehicle

        # Extract reference horizon for controller
        ref_window = planner.get_reference_window(veh_state[0], horizon_steps=15, dt=dt)
        curr_ref = ref_window[0]

        # Calculate tracking errors
        dx = veh_state[0] - curr_ref[0]
        dy = veh_state[1] - curr_ref[1]
        lat_error = -math.sin(curr_ref[2]) * dx + math.cos(curr_ref[2]) * dy
        yaw_error = math.atan2(math.sin(veh_state[2] - curr_ref[2]), math.cos(veh_state[2] - curr_ref[2]))
        vel_error = veh_state[3] - curr_ref[3]

        # Execute Controller (YOU)
        if controller_name.upper() == "MPC":
            applied_accel, applied_steer, comp_time_ms, status, pred_traj = controller.solve(
                current_state=veh_state,
                reference_trajectory=ref_window
            )
        else:
            applied_accel, applied_steer, comp_time_ms, status, pred_traj = controller.solve(
                current_state=veh_state,
                reference_target=curr_ref
            )

        # Log Telemetry Record (Snehal)
        record = TelemetryRecord(
            timestamp=round(t, 4),
            step=step,
            x=veh_state[0],
            y=veh_state[1],
            yaw=veh_state[2],
            v=veh_state[3],
            omega=(veh_state[3] / 1.2) * math.tan(applied_steer),
            x_ref=curr_ref[0],
            y_ref=curr_ref[1],
            yaw_ref=curr_ref[2],
            v_ref=curr_ref[3],
            lateral_error=lat_error,
            heading_error=yaw_error,
            velocity_error=vel_error,
            accel=applied_accel,
            steering_angle=applied_steer,
            mpc_comp_time_ms=comp_time_ms,
            mpc_status=status,
            min_clearance=sensor.min_clearance,
            nearest_obstacle_id=sensor.nearest_obstacle_id,
            collision_detected=sensor.collision_detected,
            collision_count=sensor.collision_count,
            active_destination_id=destination_id,
            goal_x=goal_coord[0],
            goal_y=goal_coord[1],
            replan_count=replan_count,
            replan_in_progress=replan_executed and (t < 6.0),
            route_completion_ratio=min(1.0, max(0.0, veh_state[0] / goal_coord[0]))
        )
        logger.log_step(record)

        t += dt
        step += 1

    logger.end_run(final_status="SUCCESS")
    print(f"[Done] Logged {step} steps. Log file: {logger.csv_path}")


def main():
    parser = argparse.ArgumentParser(description="SteadyPath Master Integration")
    parser.add_argument("--controller", choices=["mpc", "stanley"], default="mpc", help="Controller type")
    parser.add_argument("--scenario", choices=["normal", "curved", "static", "moving", "replan", "destA", "destB", "stress", "all"], default="normal")
    parser.add_argument("--dest", default="DEST_BAY_01", help="Destination ID")
    args = parser.parse_args()

    if args.scenario == "all":
        # Run standard battery across all 4 project deliverables
        run_integrated_simulation("exp01_normal_nav_dest1", "MPC", "DEST_BAY_01", "normal")
        run_integrated_simulation("exp02_curved_mpc", "MPC", "DEST_BAY_02", "curved")
        run_integrated_simulation("exp02_curved_stanley", "STANLEY", "DEST_BAY_02", "curved")
        run_integrated_simulation("exp03_static_obstacles", "MPC", "DEST_BAY_01", "static")
        run_integrated_simulation("exp04_moving_obstacle", "MPC", "DEST_BAY_01", "moving")
        run_integrated_simulation("exp05_dynamic_replan_mpc", "MPC", "DEST_BAY_01", "replan")
        run_integrated_simulation("exp06_multi_dest_A", "MPC", "DEST_BAY_03", "destA")
        run_integrated_simulation("exp06_multi_dest_B", "MPC", "DEST_BAY_04", "destB")
        run_integrated_simulation("exp07_stress_test", "MPC", "DEST_BAY_01", "stress")

        # Automatically recompute metrics
        print("\n--> Aggregating final metrics through Snehal's engine...")
        processor = MetricsBatchProcessor()
        processor.process_all_runs()
    else:
        run_id = f"run_{args.controller}_{args.scenario}"
        run_integrated_simulation(run_id, args.controller, args.dest, args.scenario)


if __name__ == "__main__":
    main()
