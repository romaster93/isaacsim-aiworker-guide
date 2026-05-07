#!/usr/bin/env bash
# ICROS 2026 실험용 rosbag 녹화 스크립트.
# 사용법: ./record_experiment.sh <run_name>
#   예) ./record_experiment.sh swerve_run1
#        ./record_experiment.sh ackermann_run1
# Ctrl+C 로 녹화 종료 → /home/cho/ms_AIworker/rosbags/<run_name>_<timestamp>/ 에 저장.

set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <run_name>   (e.g. swerve_run1, ackermann_run1)" >&2
  exit 1
fi

RUN_NAME="$1"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="/home/cho/ms_AIworker/rosbags/${RUN_NAME}_${TIMESTAMP}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS2 Jazzy + FastDDS UDP 환경 자동 로드.
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/ros2-bridge-env.sh"

# Fig.3 + Table1 재생성에 필요한 최소 토픽 세트.
TOPICS=(
  /habitat/odom
  /tf
  /tf_static
  /move_base_simple/goal
  /ros/expl_state
  /ros/expl_result
  /grid_map/occupied
  /grid_map/free
  /cmd_vel
  /planning_vis/trajectory
)

echo "=================================================="
echo " run name : ${RUN_NAME}"
echo " output   : ${OUT_DIR}"
echo " topics   : ${#TOPICS[@]}"
printf '   - %s\n' "${TOPICS[@]}"
echo "=================================================="
echo " 녹화 시작. 종료하려면 Ctrl+C."
echo "=================================================="

exec ros2 bag record -s mcap -o "${OUT_DIR}" "${TOPICS[@]}"
