#!/usr/bin/env bash
# set -euo pipefail

container_name="$1"
if [ -z "$container_name" ]; then
  echo "[ERR] usage: $0 <container_name>" >&2
  exit 1
fi

echo "[NAV] container=${container_name}"

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
  echo "[NAV] starting container..."
  docker start "$container_name"
fi

echo "[NAV] exec /root/cb.sh in container..."
# No -t to avoid "the input device is not a TTY" under systemd
docker exec "$container_name" bash -lc '/root/cb.sh'