FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-foxglove-bridge \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    ros-humble-desktop \
    ros-humble-urdf-tutorial \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ros2_ws/
