#!/usr/bin/env bash
# set -euo pipefail

# num parameters must not lt 3 and not gt 6
if [ "$#" -lt 3 ] || [ "$#" -gt 6 ]; then
  echo "Usage: $0 <container_name> <docker_script_path> <robot_uid> [<initial_pose_x> <initial_pose_y> <initial_pose_yaw>]" >&2
  exit 1
fi

container_name="$1"
docker_script_path="$2"   # absolute path inside container
robot_uid="$3"
initial_pose_x=0.0
initial_pose_y=0.0
initial_pose_yaw=0.0
if [ "$#" -ge 4 ]; then
  initial_pose_x="$4"
  initial_pose_y="$5"
  initial_pose_yaw="$6"
fi


echo "[NAV] container=${container_name} docker_script=${docker_script_path} robot_uid=${robot_uid} initial_pose_x=${initial_pose_x} initial_pose_y=${initial_pose_y} initial_pose_yaw=${initial_pose_yaw}"


# Ensure docker is available and daemon is up
if ! command -v docker >/dev/null 2>&1; then
  echo "[ERR] docker not found in PATH" >&2
  exit 1
fi
for i in $(seq 1 30); do
  if docker info >/dev/null 2>&1; then break; fi
  sleep 1
done

# Start the container if not running
if ! docker inspect -f '{{.State.Running}}' "$container_name" 2>/dev/null | grep -q true; then
  echo "[NAV] starting container... ${container_name}"
  docker start "$container_name"
fi

# echo "[NAV] exec /root/cb.sh in container..."
# # No -t to avoid "the input device is not a TTY" under systemd
# docker exec "$container_name" bash -lc '/root/cb.sh'

echo "[NAV] exec ${docker_script_path} '${robot_uid}' ${initial_pose_x} ${initial_pose_y} ${initial_pose_yaw} in container..."
# No -t to avoid "the input device is not a TTY" under systemd
if ! docker exec ${container_name} bash -lc "exec ${docker_script_path} ${robot_uid} ${initial_pose_x} ${initial_pose_y} ${initial_pose_yaw}"; then
  rc=$?
  echo "[ERR] docker exec failed with code $rc" >&2
  exit "$rc"
fi