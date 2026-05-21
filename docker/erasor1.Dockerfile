# ROS Melodic image with jsk visualization + catkin tools, sized to build
# and run the upstream ERASOR (v1) repo end-to-end on the KAIST rosbags
# distributed with the paper.
FROM ros:melodic-perception

ENV DEBIAN_FRONTEND=noninteractive

# Upstream's published install list: jsk-recognition / common-msgs /
# rviz-plugins + catkin tools. We also pull rosbag because the perception
# variant is mostly libpcl + libopencv, no rosbag CLI.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python-catkin-tools \
        ros-melodic-jsk-recognition \
        ros-melodic-jsk-common-msgs \
        ros-melodic-jsk-rviz-plugins \
        ros-melodic-rosbag \
    && rm -rf /var/lib/apt/lists/*

# Workspace at /catkin_ws/src; runtime mounts ERASOR source + the bag dir.
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Configure shell so each invocation has ros + catkin sourced.
SHELL ["/bin/bash", "-c"]
