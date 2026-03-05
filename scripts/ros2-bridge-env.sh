#!/bin/bash
# Docker 컨테이너의 ROS2 토픽을 호스트에서 확인하기 위한 환경 설정
# Shared Memory 대신 UDP 통신을 사용하도록 FastDDS를 설정합니다.
#
# Usage:
#   source scripts/ros2-bridge-env.sh
#
# 이후 ros2, rviz2 등 모든 ROS2 명령어를 그대로 사용할 수 있습니다.
# 설정은 현재 터미널에서만 유효하며, 새 터미널을 열면 원래대로 복원됩니다.
#
# 해제하려면:
#   unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_IMPLEMENTATION

# 프로젝트 경로 자동 감지 (source 방식과 직접 실행 모두 지원)
if [ -n "${BASH_SOURCE[0]}" ] && [ "${BASH_SOURCE[0]}" != "$0" ]; then
    # source로 실행된 경우
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    # 직접 실행된 경우
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ROS2 환경 로드
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[ros2-bridge-env] ROS2 Jazzy 로드됨"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[ros2-bridge-env] ROS2 Humble 로드됨"
else
    echo "[ros2-bridge-env] WARNING: ROS2가 설치되어 있지 않습니다"
    return 1 2>/dev/null || exit 1
fi

# FastDDS UDP 설정
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_DIR/fastdds.xml"

echo "[ros2-bridge-env] FastDDS UDP 설정 완료 (Docker 컨테이너 통신용)"
echo "[ros2-bridge-env] 이제 ros2, rviz2 명령어를 사용할 수 있습니다"
echo ""
echo "  ros2 topic list"
echo "  ros2 topic echo /tf --once"
echo "  rviz2"
echo ""
echo "해제: unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_IMPLEMENTATION"
