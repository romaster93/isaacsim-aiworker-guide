#!/bin/bash
# IsaacSim Docker 실행 스크립트
# Usage: ./scripts/docker-run.sh [gui|headless|stream|shell|stop|ros2]

set -e

MODE=${1:-gui}
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FASTDDS_XML="$PROJECT_DIR/fastdds.xml"

# GPU 확인
check_gpu() {
    if ! nvidia-smi &> /dev/null; then
        echo "[ERROR] NVIDIA GPU를 찾을 수 없습니다."
        exit 1
    fi
}

# 컨테이너 시작 (공통)
start_container() {
    cd "$PROJECT_DIR"

    # 이미지 빌드 (최초 1회, 이후 캐시 사용)
    if ! docker images | grep -q "isaac-sim-ros2"; then
        echo "[BUILD] Docker 이미지 빌드 중..."
        docker compose build
    fi

    # X11 권한
    xhost +local: 2>/dev/null || true

    # 파일 권한 (컨테이너의 isaac-sim 유저가 읽을 수 있도록)
    chmod -R o+rX "$PROJECT_DIR/isaacsim_ai_worker/" 2>/dev/null || true

    docker compose up -d

    # 볼륨 권한 수정 (Named Volume은 root로 생성됨)
    docker exec -u root isaac-sim chown -R isaac-sim:isaac-sim \
        /isaac-sim/.cache /isaac-sim/.local /isaac-sim/.nvidia-omniverse /isaac-sim/.nv \
        2>/dev/null || true
}

case "$MODE" in
    gui)
        check_gpu
        echo "=== IsaacSim GUI 모드 ==="
        start_container
        echo ""
        echo "컨테이너 접속 후 ./runapp.sh 실행:"
        echo "  docker exec -it isaac-sim bash"
        echo "  ./runapp.sh"
        echo ""
        echo "호스트에서 ROS2 토픽 확인:"
        echo "  export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_XML"
        echo "  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        echo "  ros2 topic list"
        docker exec -it isaac-sim bash
        ;;
    headless)
        check_gpu
        echo "=== IsaacSim Headless 모드 ==="
        start_container
        docker exec -it isaac-sim bash -c "./runheadless.native.sh -v"
        ;;
    stream)
        check_gpu
        echo "=== IsaacSim Livestream 모드 ==="
        echo "WebRTC 클라이언트로 접속하세요"
        start_container
        docker exec -it isaac-sim bash -c "./runheadless.sh -v"
        ;;
    shell)
        check_gpu
        echo "=== IsaacSim 컨테이너 쉘 ==="
        start_container
        docker exec -it isaac-sim bash
        ;;
    ros2)
        echo "=== ROS2 호스트 환경 설정 ==="
        echo "다음 명령어를 실행하세요 (또는 .bashrc에 추가):"
        echo ""
        echo "  export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_XML"
        echo "  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        echo ""
        echo "확인:"
        echo "  ros2 topic list"
        echo "  ros2 topic echo /tf --once"
        ;;
    stop)
        echo "=== 컨테이너 중지 ==="
        cd "$PROJECT_DIR"
        docker compose down
        ;;
    *)
        echo "Usage: $0 [gui|headless|stream|shell|ros2|stop]"
        echo ""
        echo "  gui      - GUI 모드 (모니터에 IsaacSim 창 표시)"
        echo "  headless - Headless 모드 (Python 스크립트 전용)"
        echo "  stream   - Livestream 모드 (WebRTC로 원격 접속)"
        echo "  shell    - 컨테이너 쉘 접속"
        echo "  ros2     - 호스트 ROS2 환경 설정 안내"
        echo "  stop     - 컨테이너 중지"
        exit 1
        ;;
esac
