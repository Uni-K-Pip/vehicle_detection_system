#!/usr/bin/env bash
# One-command demo runner for vehicle_detection_system.
#
# Usage (from the repository root, inside a ROS 2 Jazzy environment):
#   ./tools/run_demo.sh [PCD_PATH] [ROS_LAUNCH_ARGS...]
#
# What it does:
#   1. Sources /opt/ros/jazzy/setup.bash if available.
#   2. Builds the workspace with colcon (--merge-install).
#   3. Sources install/setup.bash.
#   4. Launches vehicle_detection.launch.py with the provided PCD.
#
# Defaults to data/pcd/sample.pcd.
set -eo pipefail

PCD="${1:-data/pcd/sample.pcd}"
if [[ $# -gt 0 ]]; then
  shift
fi

if [[ ! -f "$PCD" ]]; then
  echo "error: PCD not found: $PCD" >&2
  echo "place a PCD at data/pcd/sample.pcd or pass an explicit path." >&2
  exit 1
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

colcon build --merge-install --packages-select vehicle_detection

set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

exec ros2 launch vehicle_detection vehicle_detection.launch.py \
  pcd_file:="$PCD" \
  publish_once:=false \
  "$@"
