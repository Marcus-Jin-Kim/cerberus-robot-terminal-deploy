#!/bin/bash
bash ../../ugv_ws/ros2_humble.sh
# get docker container name
CONTAINER_NAME=$(docker ps --filter "name=ros_humble" --format "{{.Names}}")
if [ -z "$CONTAINER_NAME" ]; then
  echo "No running container found from ros:humble image. Please start the container first."
  exit 1
fi
echo "Found container from ros:humble image: $CONTAINER_NAME"
docker exec -it $CONTAINER_NAME bash -c "cd /root && source .bashrc && cd /home/ws/ugv_ws && source install/setup.bash && colcon build --symlink-install --packages-select ugv_nav"