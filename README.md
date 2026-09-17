# SteadyPath: Autonomous Guided Vehicle (AGV) Motion Control & ROS 2 Deployment

High-precision autonomous motion control, real-time convex Model Predictive Control (MPC), dynamic replanning, 3D warehouse simulation, and publication-grade telemetry analytics.

---

## Repository Overview

- **Team Allocation**:
  - **MPC & Lead Integration**: Antigravity / Team Lead (`mpc/`, `integrate.py`)
  - **Global Route Planning & Dynamic Replanning**: R.P. Singh (`planner/`)
  - **3D Warehouse Simulation & Dynamic Obstacles**: B. Sheshank (`simulation/`)
  - **Telemetry Logging, Metrics & Experimental Proofs**: Snehal (`analysis/`)
  - **Track A: ROS 2 Humble Production Deployment**: `steadypath_ros2/`

---

## Architecture & Subsystems

```
steadypath/
├── mpc/                          # Real-Time LTV-MPC & Stanley baseline controllers
├── planner/                      # Curvature-continuous cubic spline planner & replanner
├── simulation/                   # 50 Hz kinematic AGV physics & dynamic obstacle engine
├── analysis/                     # 14-day simulation logs, 300 DPI plots, IEEE LaTeX tables
├── steadypath_ros2/              # Production ROS 2 Humble deployment package
│   ├── package.xml               # ament_python format 3 manifest
│   ├── setup.py / setup.cfg      # Console script node entrypoints
│   ├── Dockerfile                # Zero-dependency containerized runtime
│   ├── docker-compose.yml        # Single-command stack execution
│   ├── config/                   # mpc_params.yaml & rviz_config.rviz
│   ├── launch/                   # steadypath.launch.py master launcher
│   ├── urdf/                     # Industrial AGV 3D visual & collision model
│   └── steadypath_ros2/          # ROS 2 Nodes: mpc, planner, sim_bridge, telemetry
├── tests/                        # 26 automated unit & integration tests
├── integrate.py                  # Master CLI integration engine
├── PROJECT_REPORT.md             # Comprehensive academic technical report
└── PRESENTATION_SLIDES.md        # 15-slide viva defense presentation deck
```

---

## Track A: Production ROS 2 Deployment

The `steadypath_ros2` package bridges all core modules into standard ROS 2 interfaces:

| Node | Subscribed Topics | Published Topics | Loop Rate |
| :--- | :--- | :--- | :--- |
| **`mpc_node`** | `/odom`<br>`/global_plan` | `/cmd_vel` (`geometry_msgs/Twist`)<br>`/mpc_predicted_path` (`nav_msgs/Path`)<br>`/steadypath/control_status` | 50 Hz |
| **`planner_node`** | `/goal_pose`<br>`/scan`<br>`/odom` | `/global_plan` (`nav_msgs/Path`)<br>`/steadypath/replan_event` | 2 Hz |
| **`sim_bridge_node`** | `/cmd_vel` | `/odom` (`nav_msgs/Odometry`)<br>`/scan` (`sensor_msgs/LaserScan`)<br>TF: `odom -> base_link -> laser_frame` | 50 Hz |
| **`telemetry_logger_node`** | `/odom`<br>`/cmd_vel`<br>`/steadypath/*` | Streams Snehal's 32-column CSV/JSONL records & evaluates metrics on shutdown | 50 Hz |

### Instant Launch with Docker Compose
```bash
cd steadypath_ros2
docker compose up
```

### Native Build with colcon
```bash
cd ~/ros2_ws
colcon build --packages-select steadypath_ros2 --symlink-install
source install/setup.bash
ros2 launch steadypath_ros2 steadypath.launch.py scenario:=moving destination_id:=DEST_BAY_01
```

---

## Experimental Benchmarks & Proofs

1. **Dynamic Destination Selection**: Seamlessly switches across warehouse loading bays (`DEST_BAY_01` to `04`) with $< 0.05\,\text{m}$ endpoint tolerance.
2. **Dynamic Route Replanning**: Detects moving forklifts and corridor blockages via 2D LiDAR, generating curvature-continuous detour trajectories in real time without AGV emergency stalls.
3. **Control Superiority (MPC vs Stanley)**:
   - **97.0%** reduction in RMS lateral error ($0.0043\,\text{m}$ vs $0.1432\,\text{m}$).
   - **96.5%** reduction in steering jerk integral.
   - **$< 5\,\text{ms}$** analytical MPC solver cycle latency, well within the 20 ms (50 Hz) budget.

---

## Running Verification Tests

Run the complete 26-test suite across kinematics, MPC, route planner, simulation bridge, ROS 2 message converters, and telemetry contracts:
```powershell
.\.venv\Scripts\python.exe tests\test_all.py
```
*(All 26 tests execute and pass in $< 0.1\,\text{s}$)*
