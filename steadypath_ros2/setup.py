import os
from glob import glob
from setuptools import setup, find_packages

package_name = "steadypath_ros2"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.*"),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*.*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Basappa Babanagar",
    maintainer_email="basappababanagar911@example.com",
    description="SteadyPath AGV Motion Control and Dynamic Replanning ROS 2 Package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_node = steadypath_ros2.mpc_node:main",
            "planner_node = steadypath_ros2.planner_node:main",
            "sim_bridge_node = steadypath_ros2.sim_bridge_node:main",
            "telemetry_logger_node = steadypath_ros2.telemetry_logger_node:main",
        ],
    },
)
