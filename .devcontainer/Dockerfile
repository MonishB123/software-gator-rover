FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-foxglove-bridge \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    ros-humble-desktop \
    ros-humble-xacro \
    python3 \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install opencv-python

WORKDIR /app

COPY . .