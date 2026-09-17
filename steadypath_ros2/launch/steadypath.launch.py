"""
steadypath.launch.py
====================
Unified ROS 2 Launch File for SteadyPath AGV Motion Control System.

Brings up:
1. steadypath_sim_bridge_node (Kinematics, 3D Warehouse racks, moving forklifts, LiDAR)
2. steadypath_planner_node (Global spline route generator & dynamic replanner)
3. steadypath_mpc_node (50 Hz Convex LTV-MPC controller with predicted horizon publisher)
4. steadypath_telemetry_logger_node (Snehal's 32-column streaming logger and metrics)
5. robot_state_publisher (Publishes AGV 3D visual & collision model)
6. tf2_ros static_transform_publisher (Publishes map -> odom world frame)
7. rviz2 (Pre-configured 3D visualization layout)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("steadypath_ros2")

    default_params_file = os.path.join(pkg_share, "config", "mpc_params.yaml")
    default_rviz_file = os.path.join(pkg_share, "config", "rviz_config.rviz")
    urdf_file = os.path.join(pkg_share, "urdf", "agv_vehicle.urdf")

    # Read URDF file contents
    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    # Launch Configurations
    use_rviz = LaunchConfiguration("use_rviz")
    scenario = LaunchConfiguration("scenario")
    destination_id = LaunchConfiguration("destination_id")
    params_file = LaunchConfiguration("params_file")

    # Arguments
    declare_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz2 for 3D monitoring"
    )
    declare_scenario = DeclareLaunchArgument(
        "scenario", default_value="moving", description="Scenario: 'normal', 'replan', 'moving'"
    )
    declare_dest = DeclareLaunchArgument(
        "destination_id", default_value="DEST_BAY_01", description="Target warehouse destination bay"
    )
    declare_params = DeclareLaunchArgument(
        "params_file", default_value=default_params_file, description="Path to parameters YAML file"
    )

    # 1. Map to Odom Static Transform
    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # 2. Robot State Publisher (URDF)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
    )

    # 3. Simulation Bridge Node
    sim_bridge_node = Node(
        package="steadypath_ros2",
        executable="sim_bridge_node",
        name="steadypath_sim_bridge_node",
        output="screen",
        parameters=[params_file, {"scenario": scenario}],
    )

    # 4. Route Planner & Replanner Node
    planner_node = Node(
        package="steadypath_ros2",
        executable="planner_node",
        name="steadypath_planner_node",
        output="screen",
        parameters=[params_file, {"destination_id": destination_id}],
    )

    # 5. MPC Controller Node
    mpc_node = Node(
        package="steadypath_ros2",
        executable="mpc_node",
        name="steadypath_mpc_node",
        output="screen",
        parameters=[params_file],
    )

    # 6. Telemetry Logger Node
    telemetry_node = Node(
        package="steadypath_ros2",
        executable="telemetry_logger_node",
        name="steadypath_telemetry_logger_node",
        output="screen",
        parameters=[params_file],
    )

    # 7. RViz2 Display Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", default_rviz_file],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(declare_scenario)
    ld.add_action(declare_dest)
    ld.add_action(declare_params)

    ld.add_action(static_map_odom)
    ld.add_action(robot_state_pub)
    ld.add_action(sim_bridge_node)
    ld.add_action(planner_node)
    ld.add_action(mpc_node)
    ld.add_action(telemetry_node)
    ld.add_action(rviz_node)

    return ld
