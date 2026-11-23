# slam_toolbox on ROS kilted, like the other containers here (start.sh --slam builds it once as "edurob-slam").
# The older slam-toolbox image (../ROS-Laser-SLAM) is ROS lyrical: its /tf and /map don't reach kilted nodes.
FROM osrf/ros:kilted-desktop
RUN apt-get update && apt-get install -y --no-install-recommends ros-kilted-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*
