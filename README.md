# SteadyPath: Autonomous Guided Vehicle (AGV) Motion Control & ROS 2 Deployment

[![Tests](https://img.shields.io/badge/tests-38%2F38%20passing-brightgreen.svg)](#running-verification-tests)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](requirements.txt)
[![ROS 2](https://img.shields.io/badge/ros2-Humble%20Hawksbill-orange.svg)](steadypath_ros2/)
[![Simulation](https://img.shields.io/badge/3D%20Digital%20Twin-Three.js-purple.svg)](steadypath_3d.html)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](README.md)

**SteadyPath** is a production-grade autonomous warehouse motion control architecture featuring real-time Linear Time-Varying Model Predictive Control (LTV-MPC), curvature-continuous cubic spline dynamic replanning, 50 Hz kinematic Runge-Kutta simulation, interactive 3D digital twin visualization, and standard ROS 2 Humble robotics interfaces.

---

## 📑 Repository Structure

```
SteadyPath/
├── index.html                         # Interactive 3D Digital Twin (Browser Entrypoint)
├── steadypath_3d.html                 # Complete Three.js 3D Warehouse Simulator
├── integrate.py                       # Root launcher for CLI closed-loop integration
├── requirements.txt                   # Minimal Python dependencies
├── README.md                          # Production documentation & setup guide
├── PROJECT_REPORT.md                  # Comprehensive IEEE-format technical report
├── PRESENTATION_SLIDES.md             # Defense & presentation slide deck
│
├── mpc/                               # Real-Time LTV-MPC & Baseline Controllers
│   ├── kinematic_model.py             # 4-DOF bicycle kinematics with RK4 numerical integrator
│   ├── mpc_controller.py              # QP / Convex LTV-MPC with receding horizon & rate limits
│   ├── stanley_controller.py          # Classical geometric cross-track error controller
│   └── __init__.py
│
├── planner/                           # Route Planning & Dynamic Obstacle Replanning
│   ├── route_planner.py               # Curvature-continuous cubic spline global path generator
│   └── __init__.py
│
├── simulation/                        # 50 Hz Physics Environment & Assets
│   ├── warehouse_env.py               # Multi-aisle warehouse environment & obstacle tracking
│   ├── assets/                        # Offline 3D library assets (Three.js & OrbitControls)
│   └── __init__.py
│
├── scripts/                           # Simulation Launchers & Visualizers
│   ├── integrate.py                   # Master integration engine with multi-scenario runner
│   └── visualize_simulation.py        # Desktop 50 Hz animated real-time strip chart GUI
│
├── analysis/                          # Telemetry, Benchmarking & Experimental Verification
│   ├── comparison/                    # Head-to-head Stanley vs MPC benchmark report
│   ├── final_results/                 # Results summaries & verified IEEE LaTeX tables
│   ├── graphs/                        # 6 Publication-quality 300 DPI analytical figures
│   ├── metrics/                       # Aggregated summary_table.csv & metric JSON reports
│   ├── raw_data/                      # 9 Canonical experiment datasets (exp01–exp07 CSV/meta)
│   └── src/                           # Metric calculation, evaluation & plotting engine
│
├── tests/                             # Automated Test & Quality Assurance Suite
│   ├── test_all.py                    # 38 unit & contract verification tests
│   ├── test_ros2_nodes.py             # ROS 2 message converters & node lifecycle tests
│   └── __init__.py
│
└── steadypath_ros2/                   # Production ROS 2 Humble Deployment Package
    ├── Dockerfile                     # Zero-dependency containerized runtime
    ├── docker-compose.yml             # Single-command ROS 2 stack execution
    ├── package.xml                    # ament_python format 3 manifest
    ├── setup.py / setup.cfg           # Console script entrypoints
    ├── config/                        # mpc_params.yaml & rviz_config.rviz
    ├── launch/                        # steadypath.launch.py master launch file
    ├── urdf/                          # AGV 3D visual & collision model
    └── steadypath_ros2/               # ROS 2 Nodes (mpc, planner, sim_bridge, telemetry)
```

---

## 🚀 Quickstart Guide

### 1. Launch 3D Warehouse Digital Twin
Open [index.html](index.html) or [steadypath_3d.html](steadypath_3d.html) directly in any modern web browser:
- Real-time 3D rendered cube warehouse with racks, bays, pallets, and forklift traffic.
- Multi-destination dispatching: Bay 1, Bay 2, Bay 3, Bay 4, and Docking Stations.
- Real-time obstacle avoidance, dynamic path replanning, and cargo pickup/drop sequences.
- Interactive camera controls (Orbit, Top-Down, Chase Cam, Cab View).

### 2. Run CLI Closed-Loop Simulation
Execute the master integration engine across any scenario:
```bash
# Nominal navigation to Bay 1 using MPC
python integrate.py --controller mpc --scenario normal

# Dynamic replanning detour around sudden corridor blockage
python integrate.py --controller mpc --scenario replan

# Head-to-head curved path tracking with Stanley baseline
python integrate.py --controller stanley --scenario curved

# Run all 9 canonical experimental benchmark runs
python integrate.py --controller mpc --scenario all
```

### 3. Run Real-Time Desktop Graphical Visualizer
```bash
python scripts/visualize_simulation.py --controller mpc --scenario replan
```

---

## 🧪 Automated Testing & Verification

SteadyPath maintains 100% test pass rate across vehicle kinematics, MPC numerical bounds, path splines, collision checking, telemetry contracts, and ROS 2 node interfaces:

```bash
python -m unittest discover tests
```
```
......................................
----------------------------------------------------------------------
Ran 38 tests in 0.160s

OK
```

---

## 📊 Experimental Results & Benchmarks

| Metric | Stanley Baseline | SteadyPath LTV-MPC | Improvement |
| :--- | :--- | :--- | :--- |
| **RMS Lateral Error** | $0.143\,\text{m}$ | **$0.004\,\text{m}$** | **$97.0\%$ reduction** |
| **Max Lateral Error** | $0.246\,\text{m}$ | **$0.007\,\text{m}$** | **$97.2\%$ reduction** |
| **RMS Heading Error** | $1.910^\circ$ | **$0.180^\circ$** | **$90.8\%$ reduction** |
| **Steering Jerk Integral** | $0.910\,\text{rad}/\text{s}^3$ | **$0.032\,\text{rad}/\text{s}^3$** | **$96.5\%$ reduction** |
| **Solver Cycle Latency** | N/A (Geometric) | **$10.4\,\text{ms}$** | Real-time feasible ($< 50\,\text{ms}$) |
| **Terminal Goal Error** | $0.128\,\text{m}$ | **$0.079\,\text{m}$** | Precision docking compliant |
| **Route Completion Ratio**| $100.0\%$ | **$100.0\%$** | Zero mission failures |

All verified raw telemetry datasets are archived in [`analysis/raw_data/`](analysis/raw_data/) and publication figures in [`analysis/graphs/`](analysis/graphs/).

---

## 🤖 ROS 2 Humble Production Deployment

The `steadypath_ros2` package bridges all algorithmic components into standard ROS 2 nodes:

| Node | Subscribed Topics | Published Topics | Rate |
| :--- | :--- | :--- | :--- |
| **`mpc_node`** | `/odom`, `/global_plan` | `/cmd_vel`, `/mpc_predicted_path`, `/steadypath/control_status` | 50 Hz |
| **`planner_node`** | `/goal_pose`, `/scan`, `/odom` | `/global_plan`, `/steadypath/replan_event` | 2 Hz |
| **`sim_bridge_node`** | `/cmd_vel` | `/odom`, `/scan`, TF: `odom -> base_link -> laser_frame` | 50 Hz |
| **`telemetry_logger_node`** | `/odom`, `/cmd_vel`, `/steadypath/*` | 32-column telemetry CSV streams & metric summary | 50 Hz |

### Docker Deployment:
```bash
cd steadypath_ros2
docker compose up
```

### Colcon Build:
```bash
cd ~/ros2_ws
colcon build --packages-select steadypath_ros2 --symlink-install
source install/setup.bash
ros2 launch steadypath_ros2 steadypath.launch.py scenario:=replan destination_id:=DEST_BAY_01
```

---

## 👥 Contributors & Module Ownership

- **Lead Integration & LTV-MPC**: Antigravity / Team Lead (`mpc/`, `scripts/`, `integrate.py`)
- **Global Path Planning & Splines**: R.P. Singh (`planner/`)
- **Warehouse Simulation & 3D Twin**: B. Sheshank (`simulation/`, `index.html`, `steadypath_3d.html`)
- **Telemetry, Evaluation & LaTeX Package**: Snehal (`analysis/`)
- **Track A ROS 2 Architecture**: Production Engineering Team (`steadypath_ros2/`)
