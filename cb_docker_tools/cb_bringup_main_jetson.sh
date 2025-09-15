#!/bin/bash

ROBOT_UID="$1"
if [ -z "$ROBOT_UID" ]; then
  echo "[ERR] usage: $0 <robot_uid>" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash
export UGV_MODEL=ugv_rover
export LDLIDAR_MODEL=ld19
export ROS_DOMAIN_ID=33
#ros2 launch ugv_slam rtabmap_rgbd.launch.py use_rviz:=false
#ros2 launch ugv_nav slam_nav.launch.py use_rviz:=false use_namespace:=true namespace:=beast001
#ros2 launch ugv_nav CB_slam_nav.py use_rviz:=false use_namespace:=true namespace:=beast001
#ros2 launch ugv_bringup CB_bringup_lidar.launch.py use_namespace:=true namespace:=beast001
ros2 launch ugv_nav CB_main_launch.py namespace:=${ROBOT_UID}