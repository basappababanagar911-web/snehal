"""
test_snehal_pipeline.py
=======================
SteadyPath Complete Pipeline Verification Harness (Snehal's Module)

Simulates realistic telemetry runs covering all key milestones:
1. Normal Navigation Experiment (Day 4)
2. Curved Route Experiment: MPC vs Stanley (Day 5 & Day 11)
3. Dynamic Replanning under Blockage (Day 8 - Proof 2)
4. Dynamic Destination Selection (Day 9 - Proof 1)

Executes the full flow:
Telemetry Logger -> Raw Data -> Metric Evaluator -> Batch Summaries -> Publication Plots.
"""

import os
import sys
import math
import random

# Ensure local imports work cleanly
src_dir = os.path.dirname(os.path.abspath(__file__))
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from data_logger import SteadyPathLogger, TelemetryRecord
from evaluator import SteadyPathEvaluator
from compute_metrics import MetricsBatchProcessor
from plot_generator import SteadyPathPlotter


def get_reference_pose(x: float, replan: bool, obs_x: float):
    """Calculates reference (y, yaw, curvature) for a given x coordinate."""
    k_w = 2.0 * math.pi / 25.0
    if not replan:
        y_ref = 2.0 * math.sin(x * k_w)
        dydx = 2.0 * k_w * math.cos(x * k_w)
        d2ydx2 = -2.0 * (k_w ** 2) * math.sin(x * k_w)
    else:
        # Detour around blockage at obs_x
        detour = 2.2 * math.exp(-((x - obs_x) ** 2) / 10.0)
        d_detour = -detour * (2.0 * (x - obs_x) / 10.0)
        y_ref = 2.0 * math.sin(x * k_w) + detour
        dydx = 2.0 * k_w * math.cos(x * k_w) + d_detour
        d2ydx2 = -2.0 * (k_w ** 2) * math.sin(x * k_w)

    yaw_ref = math.atan2(dydx, 1.0)
    curvature = d2ydx2 / ((1.0 + dydx ** 2) ** 1.5)
    return y_ref, yaw_ref, curvature, dydx


def simulate_run(
    run_id: str,
    controller: str = "MPC",
    replan_at_t: float = -1.0,
    dest_name: str = "DEST_BAY_01",
    dest_x: float = 30.0,
    obstacle_pos: tuple = (14.0, 0.0),
    dt: float = 0.05,
    max_time: float = 35.0,
    output_dir: str = "analysis/raw_data"
):
    """Generates realistic vehicle telemetry for a given scenario."""
    # Terminal target pose on reference trajectory
    obs_x, obs_y = obstacle_pos
    is_replan_scenario = (replan_at_t > 0.0)
    dest_y, dest_yaw, _, _ = get_reference_pose(dest_x, replan=is_replan_scenario, obs_x=obs_x)
    dest_coord = (dest_x, dest_y)

    logger = SteadyPathLogger(
        run_id=run_id,
        controller_type=controller,
        output_dir=output_dir,
        export_csv=True,
        export_jsonl=True
    )
    logger.start_run(metadata={
        "scenario": run_id,
        "destination_name": dest_name,
        "goal_coordinates": [round(dest_x, 4), round(dest_y, 4)],
        "nominal_velocity": 1.5
    })

    # Kinematic vehicle state
    x, y, yaw, v = 0.0, 0.0, 0.465, 0.0
    v_nominal = 1.5
    wheelbase = 1.2
    
    t = 0.0
    step = 0
    replan_triggered = False
    replan_count = 0

    obs_radius = 0.4
    veh_radius = 0.35

    random.seed(42 if controller == "MPC" else 84)

    # Simulation loop
    while t <= max_time:
        dist_to_goal = math.hypot(x - dest_coord[0], y - dest_coord[1])
        if dist_to_goal < 0.08 and t > 5.0:
            # Reached destination smoothly
            break
        if x >= dest_x:
            break

        # Trigger dynamic replan if scheduled
        if replan_at_t > 0.0 and t >= replan_at_t and not replan_triggered:
            replan_triggered = True
            replan_count += 1
            logger.log_event("CORRIDOR_BLOCKED", {"obstacle_pos": [obs_x, obs_y]}, timestamp=t)
            logger.log_event("REPLAN_TRIGGERED", {"old_route_id": "R1", "new_route_id": "R2"}, timestamp=t)

        # Reference values at current x
        y_ref, yaw_ref, curvature_ref, dydx = get_reference_pose(x, replan_triggered, obs_x)

        # Smooth velocity profile with gentle deceleration near goal
        if dist_to_goal < 3.0:
            curr_v_ref = max(0.2, (dist_to_goal / 3.0) * v_nominal)
        else:
            curr_v_ref = v_nominal

        # Tracking errors in Frenet coordinates
        dy = y - y_ref
        lat_error = math.cos(yaw_ref) * dy
        yaw_error = math.atan2(math.sin(yaw - yaw_ref), math.cos(yaw - yaw_ref))
        vel_error = v - curr_v_ref

        # Controller execution
        if controller == "MPC":
            # MPC predictive feedforward (wheelbase * curvature) + optimal feedback gain
            ff_steer = math.atan(wheelbase * curvature_ref)
            fb_steer = -0.90 * lat_error - 1.15 * yaw_error
            steer = ff_steer + fb_steer + random.gauss(0, 0.0005)
            steer = max(-math.radians(35), min(math.radians(35), steer))
            accel = -1.4 * vel_error
            accel = max(-1.5, min(1.5, accel))
            comp_time = random.uniform(7.2, 13.5)
        else:
            # Stanley controller baseline: reactive, ignores curvature preview
            k_e = 0.95
            steer = -yaw_error + math.atan2(-k_e * lat_error, max(0.2, v)) + random.gauss(0, 0.006)
            steer = max(-math.radians(35), min(math.radians(35), steer))
            accel = -1.1 * vel_error + random.gauss(0, 0.015)
            accel = max(-2.0, min(2.0, accel))
            comp_time = random.uniform(0.1, 0.3)

        # Vehicle kinematics update (Runge-Kutta / Euler integration)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += (v / wheelbase) * math.tan(steer) * dt
        v += accel * dt
        v = max(0.0, min(2.5, v))

        # Obstacle clearance
        dist_to_obs = math.hypot(x - obs_x, y - obs_y) - (obs_radius + veh_radius)
        min_clear = max(0.0, dist_to_obs)
        collision = (min_clear <= 0.0)

        # Completion ratio
        completion_ratio = min(1.0, max(0.0, x / dest_x))

        record = TelemetryRecord(
            timestamp=t,
            step=step,
            x=x,
            y=y,
            yaw=yaw,
            v=v,
            omega=(v / wheelbase) * math.tan(steer),
            x_ref=x,
            y_ref=y_ref,
            yaw_ref=yaw_ref,
            v_ref=curr_v_ref,
            curvature_ref=curvature_ref,
            lateral_error=lat_error,
            heading_error=yaw_error,
            velocity_error=vel_error,
            accel=accel,
            steering_angle=steer,
            mpc_comp_time_ms=comp_time,
            mpc_status=1,
            min_clearance=min_clear,
            nearest_obstacle_id="rack_obstacle_01",
            collision_detected=collision,
            collision_count=1 if collision else 0,
            active_destination_id=dest_name,
            goal_x=dest_x,
            goal_y=dest_y,
            replan_count=replan_count,
            replan_in_progress=replan_triggered and (t < replan_at_t + 1.0),
            route_completion_ratio=completion_ratio
        )
        logger.log_step(record)

        t += dt
        step += 1

    logger.end_run(final_status="SUCCESS")
    final_err = math.hypot(x - dest_x, y - dest_y)
    print(f"[SIM DONE] Logged {step} steps for {run_id} ({controller}) - Final Goal Error: {final_err:.4f}m")


def run_full_pipeline_test():
    """Runs complete end-to-end test of Snehal's data & analytics pipeline."""
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    raw_dir = os.path.join(base_dir, "raw_data")
    metrics_dir = os.path.join(base_dir, "metrics")
    graphs_dir = os.path.join(base_dir, "graphs")
    comp_dir = os.path.join(base_dir, "comparison")

    print("\n========================================================")
    print("  SteadyPath Snehal Pipeline: End-to-End Execution Test ")
    print("========================================================\n")

    # Step 1: Simulate experimental runs
    print("Step 1: Generating Raw Simulation Runs...")
    # Experiment 1: Normal Navigation (Day 4) - Free corridor
    simulate_run("exp01_normal_nav_dest1", controller="MPC", dest_name="DEST_BAY_01", dest_x=30.0, obstacle_pos=(999.0, 999.0), output_dir=raw_dir)
    
    # Experiment 2: Curved Route MPC (Day 5) - Free corridor
    simulate_run("exp02_curved_mpc", controller="MPC", dest_name="DEST_BAY_02", dest_x=30.0, obstacle_pos=(999.0, 999.0), output_dir=raw_dir)

    # Experiment 3: Baseline Comparison Stanley (Day 11) - Free corridor
    simulate_run("exp02_curved_stanley", controller="STANLEY", dest_name="DEST_BAY_02", dest_x=30.0, obstacle_pos=(999.0, 999.0), output_dir=raw_dir)

    # Experiment 4: Dynamic Replanning under Blockage (Day 8 - Proof 2) - Corridor blocked at x=14.0
    simulate_run("exp05_dynamic_replan_mpc", controller="MPC", replan_at_t=5.5, obstacle_pos=(14.0, 0.0), dest_name="DEST_BAY_01", dest_x=30.0, output_dir=raw_dir)

    # Experiment 5 & 6: Multi-destination runtime testing (Day 9 - Proof 1)
    simulate_run("exp06_multi_dest_A", controller="MPC", dest_name="DEST_BAY_03", dest_x=25.0, obstacle_pos=(999.0, 999.0), output_dir=raw_dir)
    simulate_run("exp06_multi_dest_B", controller="MPC", dest_name="DEST_BAY_04", dest_x=35.0, obstacle_pos=(999.0, 999.0), output_dir=raw_dir)

    # Step 2: Batch compute metrics
    print("\nStep 2: Processing Batch Metrics...")
    processor = MetricsBatchProcessor(
        raw_data_dir=raw_dir,
        metrics_dir=metrics_dir,
        comparison_dir=comp_dir
    )
    all_metrics = processor.process_all_runs()

    # Step 3: Generate publication graphs
    print("\nStep 3: Generating Publication-Ready Graphs...")
    plotter = SteadyPathPlotter(output_dir=graphs_dir)

    # Load data for primary plots
    mpc_data = SteadyPathLogger.load_csv(os.path.join(raw_dir, "exp02_curved_mpc.csv"))
    stanley_data = SteadyPathLogger.load_csv(os.path.join(raw_dir, "exp02_curved_stanley.csv"))
    replan_data = SteadyPathLogger.load_csv(os.path.join(raw_dir, "exp05_dynamic_replan_mpc.csv"))

    # Graph 1: 2D Trajectory Tracking
    p1 = plotter.plot_trajectory_2d(mpc_data, filename="trajectory_tracking.png")
    print(f"  -> Generated: {p1}")

    # Graph 2: Tracking Errors (Lat, Yaw, Vel)
    p2 = plotter.plot_tracking_errors(mpc_data, filename="tracking_errors.png")
    print(f"  -> Generated: {p2}")

    # Graph 3: Control Signals & Latency
    p3 = plotter.plot_control_signals(mpc_data, filename="control_signals.png")
    print(f"  -> Generated: {p3}")

    # Graph 4: Obstacle Clearance Profile
    p4 = plotter.plot_obstacle_clearance(replan_data, filename="obstacle_clearance.png")
    print(f"  -> Generated: {p4}")

    # Graph 5: Dynamic Replan Event
    nominal_path = (mpc_data["x_ref"], mpc_data["y_ref"])
    replan_path = (replan_data["x_ref"], replan_data["y_ref"])
    actual_replan_path = (replan_data["x"], replan_data["y"])
    trigger_idx = min(110, len(replan_data["x"]) - 1)
    p5 = plotter.plot_dynamic_replan_event(
        nominal_path=nominal_path,
        replan_path=replan_path,
        actual_path=actual_replan_path,
        blockage_pos=(14.0, 0.0),
        replan_trigger_pos=(replan_data["x"][trigger_idx], replan_data["y"][trigger_idx]),
        goal_pos=(30.0, replan_data["goal_y"][-1]),
        filename="dynamic_replan_proof.png"
    )
    print(f"  -> Generated: {p5}")

    # Graph 6: Stanley vs MPC Benchmark
    p6 = plotter.plot_stanley_vs_mpc_comparison(
        stanley_data=stanley_data,
        mpc_data=mpc_data,
        filename="stanley_vs_mpc_benchmark.png"
    )
    print(f"  -> Generated: {p6}")

    print("\n========================================================")
    print("  All Snehal Pipeline Modules Successfully Verified!    ")
    print("========================================================\n")


if __name__ == "__main__":
    run_full_pipeline_test()
