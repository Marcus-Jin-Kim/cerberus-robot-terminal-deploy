#!/bin/bash
export UGV_MODEL=ugv_beast # this is only reason for different from rpi script
export LDLIDAR_MODEL=ld19


ROBOT_UID="$1"
if [ -z "$ROBOT_UID" ]; then
  echo "[ERR] usage: $0 <robot_uid> <robot_domain_id> [<initial_pose_x> <initial_pose_y> <initial_pose_yaw>]" >&2
  exit 1
fi

ROBOT_ROS_DOMAIN_ID="$2"
if [ -z "$ROBOT_ROS_DOMAIN_ID" ]; then
  echo "[ERR] usage: $0 <robot_uid> <robot_domain_id> [<initial_pose_x> <initial_pose_y> <initial_pose_yaw>]" >&2
  exit 1
fi

export ROS_DOMAIN_ID=$ROBOT_ROS_DOMAIN_ID

# export ROBOT_UID="$ROBOT_UID"
INITIAL_POSE_X=0.0
INITIAL_POSE_Y=0.0
INITIAL_POSE_YAW=0.0

# if has 5 parameters, means initial pose is provided
if [ "$#" -eq 5 ]; then
  INITIAL_POSE_X="$2"
  INITIAL_POSE_Y="$3"
  INITIAL_POSE_YAW="$4"
  echo "[INFO] initial pose provided: x=${INITIAL_POSE_X} y=${INITIAL_POSE_Y} yaw=${INITIAL_POSE_YAW}"
  # export INITIAL_POSE_X="$INITIAL_POSE_X"
  # export INITIAL_POSE_Y="$INITIAL_POSE_Y"
  # export INITIAL_POSE_YAW="$INITIAL_POSE_YAW"
fi

source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash


#ros2 launch ugv_slam rtabmap_rgbd.launch.py use_rviz:=false
#ros2 launch ugv_nav slam_nav.launch.py use_rviz:=false use_namespace:=true namespace:=beast001
#ros2 launch ugv_nav CB_slam_nav.py use_rviz:=false use_namespace:=true namespace:=beast001
#ros2 launch ugv_bringup CB_bringup_lidar.launch.py use_namespace:=true namespace:=beast001
# parameters for CB_main_launch.py
#     pub_odom_tf = LaunchConfiguration('pub_odom_tf')
#     initial_pose_x = LaunchConfiguration('initial_pose_x')
#     initial_pose_y = LaunchConfiguration('initial_pose_y')
#     initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

echo "[INFO] launching ugv_nav CB_main_launch.py with namespace=${ROBOT_UID}, initial_pose.x=${INITIAL_POSE_X}, initial_pose.y=${INITIAL_POSE_Y}, initial_pose.yaw=${INITIAL_POSE_YAW}"

ros2 launch ugv_nav CB_main_launch.py namespace:=${ROBOT_UID} \
 initial_pose.x:=${INITIAL_POSE_X} \
 initial_pose.y:=${INITIAL_POSE_Y} \
 initial_pose.yaw:=${INITIAL_POSE_YAW}