#!/bin/bash
# Docker 컨테이너의 ROS2 토픽을 호스트에서 확인하는 래퍼 스크립트
# 호스트 환경을 변경하지 않고 임시로 DDS 설정을 적용합니다.
#
# Usage:
#   ./scripts/ros2-docker.sh topic list
#   ./scripts/ros2-docker.sh topic echo /tf --once
#   ./scripts/ros2-docker.sh topic hz /tf

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_DIR/fastdds.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ROS2 환경 로드
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

exec ros2 "$@"
