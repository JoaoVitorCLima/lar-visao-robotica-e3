#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
export USER_NAME="${USER:-ros}"
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

docker compose run --rm multi_camera_robot_localization \
  bash -lc 'cd /ws/src/multi_camera_robot_localization && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m unittest discover -s tests -v'
