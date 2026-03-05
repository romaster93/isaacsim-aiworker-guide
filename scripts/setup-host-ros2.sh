#!/bin/bash
# 호스트 ROS2 환경에 FastDDS UDP 설정을 자동으로 추가합니다.
# Docker 컨테이너의 IsaacSim 토픽을 호스트에서 받기 위해 필요합니다.
#
# Usage: ./scripts/setup-host-ros2.sh
#
# 지원: ROS2 Humble (Ubuntu 22.04), ROS2 Jazzy (Ubuntu 24.04)

set -e

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FASTDDS_XML="$PROJECT_DIR/fastdds.xml"

echo "=== 호스트 ROS2 ↔ Docker 컨테이너 통신 설정 ==="
echo ""

# 1. ROS2 버전 감지
echo "[1/3] ROS2 버전 감지..."
if [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS_VER="jazzy"
    echo "  -> ROS2 Jazzy 감지됨"
elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_VER="humble"
    echo "  -> ROS2 Humble 감지됨"
else
    echo "  [ERROR] ROS2가 설치되어 있지 않습니다."
    echo "  Docker 컨테이너 안에서 ros2 명령어를 사용하거나,"
    echo "  ./scripts/ros2-docker.sh 래퍼 스크립트를 사용하세요."
    exit 1
fi

# 2. FastDDS XML 확인
echo ""
echo "[2/3] FastDDS 설정 확인..."
if [ -f "$FASTDDS_XML" ]; then
    echo "  -> $FASTDDS_XML 존재"
else
    echo "  [ERROR] $FASTDDS_XML 파일이 없습니다."
    exit 1
fi

# 3. conda 환경 감지 & 설정
echo ""
echo "[3/3] conda 환경 설정..."

CONDA_BASE=$(conda info --base 2>/dev/null || echo "")
if [ -z "$CONDA_BASE" ]; then
    echo "  conda가 설치되어 있지 않습니다."
    echo "  .bashrc에 직접 추가하거나 ros2-docker.sh 스크립트를 사용하세요."
    echo ""
    echo "  .bashrc에 추가하려면:"
    echo "    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_XML' >> ~/.bashrc"
    echo "    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc"
    exit 0
fi

# conda 환경 목록에서 isaac_sim 찾기
if conda env list | grep -q "isaac_sim"; then
    ENV_NAME="isaac_sim"
else
    echo "  isaac_sim conda 환경이 없습니다."
    echo ""
    read -p "  어떤 conda 환경에 설정할까요? (이름 입력, 또는 Enter로 건너뛰기): " ENV_NAME
    if [ -z "$ENV_NAME" ]; then
        echo "  건너뜀. ros2-docker.sh 스크립트를 사용하세요."
        exit 0
    fi
fi

ACTIVATE_DIR="$CONDA_BASE/envs/$ENV_NAME/etc/conda/activate.d"
DEACTIVATE_DIR="$CONDA_BASE/envs/$ENV_NAME/etc/conda/deactivate.d"
ACTIVATE_SCRIPT="$ACTIVATE_DIR/isaacsim_env.sh"
DEACTIVATE_SCRIPT="$DEACTIVATE_DIR/isaacsim_env.sh"

# 디렉토리 생성
mkdir -p "$ACTIVATE_DIR" "$DEACTIVATE_DIR"

# 이미 설정되어 있는지 확인
if grep -q "FASTRTPS_DEFAULT_PROFILES_FILE" "$ACTIVATE_SCRIPT" 2>/dev/null; then
    echo "  -> 이미 설정됨 (건너뜀)"
else
    # activate 스크립트가 없으면 새로 생성
    if [ ! -f "$ACTIVATE_SCRIPT" ]; then
        cat > "$ACTIVATE_SCRIPT" << ACTIVATE
#!/bin/bash
source /opt/ros/$ROS_VER/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML"
ACTIVATE
        echo "  -> activate 스크립트 생성됨: $ACTIVATE_SCRIPT"
    else
        # 기존 스크립트에 추가
        echo "export FASTRTPS_DEFAULT_PROFILES_FILE=\"$FASTDDS_XML\"" >> "$ACTIVATE_SCRIPT"
        echo "  -> activate 스크립트에 FastDDS 설정 추가됨"
    fi

    # deactivate 스크립트
    if [ ! -f "$DEACTIVATE_SCRIPT" ]; then
        cat > "$DEACTIVATE_SCRIPT" << DEACTIVATE
#!/bin/bash
unset ROS_DISTRO
unset RMW_IMPLEMENTATION
unset LD_LIBRARY_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE
DEACTIVATE
        echo "  -> deactivate 스크립트 생성됨"
    elif ! grep -q "FASTRTPS_DEFAULT_PROFILES_FILE" "$DEACTIVATE_SCRIPT"; then
        echo "unset FASTRTPS_DEFAULT_PROFILES_FILE" >> "$DEACTIVATE_SCRIPT"
        echo "  -> deactivate 스크립트에 unset 추가됨"
    fi
fi

echo ""
echo "=== 설정 완료 ==="
echo ""
echo "사용법:"
echo "  conda activate $ENV_NAME"
echo "  ros2 topic list          # Docker 컨테이너 토픽 확인"
echo "  rviz2                    # RViz2로 시각화"
echo ""
echo "호스트 ROS2 버전: $ROS_VER"
echo "FastDDS 설정: $FASTDDS_XML"
echo ""
if [ "$ROS_VER" = "humble" ]; then
    echo "[참고] 호스트가 Humble이고 Docker 컨테이너도 Humble Bridge를 사용합니다."
    echo "DDS 버전이 동일하므로 통신이 안정적입니다."
elif [ "$ROS_VER" = "jazzy" ]; then
    echo "[참고] 호스트가 Jazzy이고 Docker 컨테이너는 Humble Bridge를 사용합니다."
    echo "DDS 프로토콜 레벨에서 호환되므로 정상 통신됩니다."
fi
