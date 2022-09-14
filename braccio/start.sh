#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/poppy/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://localhost:11311
/opt/ros/noetic/bin/roslaunch braccio Lemniscate.launch
