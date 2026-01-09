# MIT License
# Copyright (c) 2020 Hongrui Zheng
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# ... (rest of MIT License text) ...

FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# System dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       libeigen3-dev \
                       ros-foxy-rviz2 \
                       nano \
                       vim \
                       tmux \
                       tree \
                       python3-pip \
                       ros-foxy-gazebo-ros \
                       ros-foxy-gazebo-ros-pkgs \
                       ros-foxy-gazebo-ros2-control \
                       ros-foxy-ros2-control \
                       ros-foxy-ros2-controllers \
                       ros-foxy-robot-state-publisher \
                       ros-foxy-joint-state-publisher \
                       ros-foxy-navigation2 \
                       ros-foxy-nav2-bringup \
                       ros-foxy-slam-toolbox && \
    apt-get -y dist-upgrade && \
    rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install --no-cache-dir transforms3d

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && pip3 install -e .

# ROS2 gym bridge
RUN mkdir -p /sim_ws/src/f1tenth_gym_ros
COPY . /sim_ws/src/f1tenth_gym_ros

# Build ROS2 workspace
RUN source /opt/ros/foxy/setup.bash && \
    cd /sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

WORKDIR /sim_ws
ENTRYPOINT ["/bin/bash"]
