#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../../" && pwd)"
COMPOSE_FILE="${COMPOSE_FILE:-${REPO_ROOT}/ros2_ws/docker/docker-compose.jetson.yml}"
SERVICE_NAME="${SERVICE_NAME:-global_gps}"
LAUNCH_ARGS="${LAUNCH_ARGS:-}"

docker compose -f "${COMPOSE_FILE}" up -d --wait "${SERVICE_NAME}"

exec docker compose -f "${COMPOSE_FILE}" exec -T "${SERVICE_NAME}" bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 launch global_gps global_gps.launch.py ${LAUNCH_ARGS}
"
