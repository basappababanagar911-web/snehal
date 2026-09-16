# SteadyPath Analysis & Results Engine
**Responsible Team Member:** Snehal  
**Mission:** Collect $\to$ Measure $\to$ Compare $\to$ Prove  

---

## 1. Directory Structure

```
analysis/
├── raw_data/                       # Standardized CSV, JSONL, and event logs per run
├── processed_data/                 # Resampled & aligned datasets
├── metrics/                        # Single-run JSONs and master summary_table.csv
├── graphs/                         # Publication-grade figures (300 DPI)
├── comparison/                     # Head-to-head MPC vs. Stanley benchmarks
├── final_results/                  # LaTeX tables and final report summaries
└── src/
    ├── data_logger.py              # Zero-overhead streaming telemetry logger
    ├── evaluator.py                # Mathematical formulations for all Day 1 metrics
    ├── compute_metrics.py          # Batch runner: raw data -> summary tables
    ├── plot_generator.py           # Publication-ready Matplotlib visualizer
    └── test_snehal_pipeline.py     # End-to-end verification and test harness
```

---

## 2. Core Modules

### `src/data_logger.py`
Implements the immutable **Day 2 Telemetry Data Contract**. Captures every simulation step with zero stall:
- State: $x, y, \psi, v, \omega$
- Reference: $x_{ref}, y_{ref}, \psi_{ref}, v_{ref}, \kappa_{ref}$
- Errors: $e_{lat}, e_{lon}, e_{\psi}, e_v$
- Controls: $a, \delta, \dot{a}, \dot{\delta}$
- MPC Diagnostics: $\tau_{comp}$, solver status, predicted horizon
- Safety: $d_{min}$, collision flags, obstacle proximity
- Planner: active destination, goal coordinates, replan counters

### `src/evaluator.py`
Implements exact mathematical definitions for all Day 1 performance metrics:
- Frenet lateral deviation $e_{lat} = -\sin(\psi_{ref})(x - x_{ref}) + \cos(\psi_{ref})(y - y_{ref})$
- Wrapped heading error $e_{\psi} \in [-\pi, \pi]$
- Control smoothness via discrete jerk integral $\int (da/dt)^2 dt$ and steering rate integral $\int (d\delta/dt)^2 dt$
- Obstacle clearance $d_{min}$ and safety buffer violations ($< 0.5\,\text{m}$)
- Goal-reaching error and route completion percentage

### `src/compute_metrics.py`
Batch processes all CSV files in `raw_data/` into:
- Individual JSON reports: `metrics/<run_id>_metrics.json`
- Master CSV table: `metrics/summary_table.csv`
- Executive Markdown report: `metrics/summary_table.md`
- Comparative benchmark: `comparison/stanley_vs_mpc_benchmark.md`

### `src/plot_generator.py`
Generates publication-quality 300-DPI visual evidence:
- `trajectory_tracking.png`: 2D XY vehicle path vs. reference
- `tracking_errors.png`: 3-panel synchronized lateral, heading, and velocity errors
- `control_signals.png`: Acceleration, steering angle, and solver latency over time
- `obstacle_clearance.png`: Proximity profile relative to the $0.5\,\text{m}$ safety threshold
- `dynamic_replan_proof.png`: Demonstration of corridor blockage and successful reroute
- `stanley_vs_mpc_benchmark.png`: Direct comparison overlays and error distribution boxplots

---

## 3. How Other Team Members Connect to Snehal

```python
from analysis.src.data_logger import SteadyPathLogger, TelemetryRecord

# Initialize logger at start of run
logger = SteadyPathLogger(run_id="exp01_demo", controller_type="MPC")
logger.start_run(metadata={"map": "warehouse_main", "target": "DEST_01"})

# Inside simulation loop (dt = 0.05s)
record = TelemetryRecord(
    timestamp=t, step=k,
    x=veh.x, y=veh.y, yaw=veh.yaw, v=veh.v,
    x_ref=ref.x, y_ref=ref.y, yaw_ref=ref.yaw, v_ref=ref.v,
    lateral_error=e_lat, heading_error=e_yaw, velocity_error=e_v,
    accel=ctrl.a, steering_angle=ctrl.delta,
    mpc_comp_time_ms=sol.time_ms,
    min_clearance=min_dist,
    active_destination_id="DEST_01",
    goal_x=goal.x, goal_y=goal.y
)
logger.log_step(record)

# Log discrete replanning event
if blockage_detected:
    logger.log_event("REPLAN_TRIGGERED", {"blockage_at": [14.0, 0.0]})

# End run
logger.end_run(final_status="SUCCESS")
```
