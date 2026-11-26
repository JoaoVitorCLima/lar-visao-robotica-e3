#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
export USER_NAME="${USER:-ros}"
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

docker compose run --rm multi_camera_robot_localization \
  roslaunch multi_camera_robot_localization fusion.launch "$@"
