# ====================================================================
# SteadyPath AGV ROS 2 Humble Production Dockerfile
# ====================================================================
FROM ros:humble-ros-base-jammy

LABEL maintainer="Basappa Babanagar <basappababanagar911@example.com>"
LABEL description="SteadyPath Autonomous Guided Vehicle (AGV) Motion Control & Replanning Stack"

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system build dependencies, Python numerical tools, and ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create colcon workspace
WORKDIR /root/ros2_ws

# Copy SteadyPath project source into workspace
COPY . /root/ros2_ws/src/steadypath/

# Build the ROS 2 package
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select steadypath_ros2 --symlink-install

# Entrypoint setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Default launch command: headless simulation + controller + logger
CMD ["bash", "-c", "source /root/ros2_ws/install/setup.bash && ros2 launch steadypath_ros2 steadypath.launch.py use_rviz:=false"]
