# SteadyPath ROS 2 Deployment Package (`steadypath_ros2`)

Production-grade ROS 2 package deploying the **SteadyPath Autonomous Guided Vehicle (AGV)** motion control stack, real-time convex Model Predictive Controller (MPC), dynamic replanning engine, 3D warehouse simulation bridge, and 32-column streaming telemetry logger.

---

## Architecture Overview

```
                          +-------------------------------+
                          |     /goal_pose (RViz2)        |
                          +---------------+---------------+
                                          |
                                          v
+------------------+             +------------------+
|   /scan (LiDAR)  | ----------> |   planner_node   |
+------------------+             +--------+---------+
                                          |
                                          | /global_plan (nav_msgs/Path)
                                          v
+------------------+   /odom     +------------------+
| sim_bridge_node  | ----------> |     mpc_node     |
+--------+---------+             +--------+---------+
         ^                                |
         |          /cmd_vel              |
         +--------------------------------+
                                          |
                                          v /mpc_predicted_path
+---------------------------------------------------+
|               telemetry_logger_node               |
|  (Enforces Snehal's 32-column streaming contract)  |
+---------------------------------------------------+
```

---

## Package Structure

```
steadypath_ros2/
├── package.xml                  # ROS 2 format 3 ament_python manifest
├── setup.py                     # Python package definitions and console script entrypoints
├── setup.cfg                    # Setuptools install configuration
├── Dockerfile                   # Ubuntu Jammy + ROS 2 Humble deployment image
├── docker-compose.yml           # Instant single-command containerized launch
├── resource/
│   └── steadypath_ros2          # Ament resource marker
├── config/
│   ├── mpc_params.yaml          # MPC gains (Q, R, Rd), constraints, sample rates
│   └── rviz_config.rviz         # Pre-tuned 3D RViz2 visualization layout
├── launch/
│   └── steadypath.launch.py     # Master launch bringing up all 5 nodes + RViz2
├── urdf/
│   └── agv_vehicle.urdf         # Visual & collision AGV model with wheels and LiDAR
└── steadypath_ros2/
    ├── __init__.py
    ├── ros_compat.py            # Mathematical quaternion/Euler & Twist converters
    ├── mpc_node.py              # 50 Hz Convex LTV-MPC controller node (< 5 ms latency)
    ├── planner_node.py          # Cubic spline route generator & dynamic replanner
    ├── sim_bridge_node.py       # 50 Hz kinematic simulation & 360° LiDAR raycaster
    └── telemetry_logger_node.py # Streaming CSV/JSONL telemetry logger
```

---

## Node Topic Interface

| Node | Subscribes | Publishes | Rate |
| :--- | :--- | :--- | :--- |
| **`mpc_node`** | `/odom`<br>`/global_plan` | `/cmd_vel` (`geometry_msgs/Twist`)<br>`/mpc_predicted_path` (`nav_msgs/Path`)<br>`/steadypath/control_status` (`std_msgs/String`) | 50 Hz |
| **`planner_node`** | `/goal_pose`<br>`/scan`<br>`/odom` | `/global_plan` (`nav_msgs/Path`)<br>`/steadypath/replan_event` (`std_msgs/String`) | 2 Hz |
| **`sim_bridge_node`** | `/cmd_vel` | `/odom` (`nav_msgs/Odometry`)<br>`/scan` (`sensor_msgs/LaserScan`)<br>TF: `odom -> base_link` | 50 Hz |
| **`telemetry_logger_node`**| `/odom`<br>`/cmd_vel`<br>`/global_plan`<br>`/steadypath/*` | Writes `analysis/telemetry_logs/<run_id>.csv` & `.jsonl` | 50 Hz |

---

## Quickstart Guide

### Option 1: Native ROS 2 Build (Ubuntu 22.04 / ROS 2 Humble)

```bash
# 1. Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# 2. Navigate to your colcon workspace
cd ~/ros2_ws

# 3. Clone or link steadypath into your workspace src
mkdir -p src
cd src
ln -s /path/to/steadypath .
cd ~/ros2_ws

# 4. Build the package
colcon build --packages-select steadypath_ros2 --symlink-install
source install/setup.bash

# 5. Launch full stack with RViz2
ros2 launch steadypath_ros2 steadypath.launch.py scenario:=moving destination_id:=DEST_BAY_01
```

### Option 2: Docker / Docker Compose (Zero Installation)

```bash
cd steadypath_ros2
docker compose build
docker compose up
```

---

## Dynamic Replanning in Action
When a dynamic obstacle (e.g. crossing forklift) enters the laser scan corridor, `planner_node` detects the obstruction, computes a Gaussian bypass trajectory around the obstacle, and issues an updated `/global_plan`. The `mpc_node` smoothly tracks the revised detour with sub-centimeter lateral accuracy without stopping the AGV.
