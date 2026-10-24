#!/usr/bin/env bash
# Launch RViz2 with the Charles River Dam inspection configuration.
#
#   ./scripts/rviz.sh
#
# Sources ROS 2 Jazzy and opens rviz2 with rviz/inspection.rviz. The repo root
# is resolved relative to this script so it works from any working directory.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck disable=SC1091
set +u; source /opt/ros/jazzy/setup.bash; set -u   # ROS setup reads unset vars

exec rviz2 -d "${REPO}/rviz/inspection.rviz"
