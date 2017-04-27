#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/LanderControls/lander_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.9.0.10:11311
export ROS_IP=10.9.0.10
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
