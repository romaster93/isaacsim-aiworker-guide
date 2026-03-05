#!/bin/bash
# IsaacSim Docker 실행 스크립트
# Usage: ./scripts/docker-run.sh [gui|headless|stream]

set -e

MODE=${1:-gui}
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# GPU 확인
if ! nvidia-smi &> /dev/null; then
    echo "[ERROR] NVIDIA GPU를 찾을 수 없습니다."
    exit 1
fi

# X11 권한 (GUI 모드)
if [[ "$MODE" == "gui" ]]; then
    xhost +local: 2>/dev/null || true
fi

case "$MODE" in
    gui)
        echo "=== IsaacSim GUI 모드 ==="
        echo "X11 디스플레이: $DISPLAY"
        cd "$PROJECT_DIR"
        docker compose up -d
        docker exec -it isaac-sim bash -c "./runapp.sh"
        ;;
    headless)
        echo "=== IsaacSim Headless 모드 (스크립트 전용) ==="
        cd "$PROJECT_DIR"
        docker compose up -d
        docker exec -it isaac-sim bash -c "./runheadless.native.sh -v"
        ;;
    stream)
        echo "=== IsaacSim Livestream 모드 ==="
        echo "WebRTC 클라이언트로 접속하세요"
        cd "$PROJECT_DIR"
        docker compose up -d
        docker exec -it isaac-sim bash -c "./runheadless.sh -v"
        ;;
    shell)
        echo "=== IsaacSim 컨테이너 쉘 ==="
        cd "$PROJECT_DIR"
        docker compose up -d
        docker exec -it isaac-sim bash
        ;;
    stop)
        echo "=== 컨테이너 중지 ==="
        cd "$PROJECT_DIR"
        docker compose down
        ;;
    *)
        echo "Usage: $0 [gui|headless|stream|shell|stop]"
        echo ""
        echo "  gui      - GUI 모드 (모니터에 IsaacSim 창 표시)"
        echo "  headless - Headless 모드 (Python 스크립트 전용)"
        echo "  stream   - Livestream 모드 (WebRTC로 원격 접속)"
        echo "  shell    - 컨테이너 쉘 접속"
        echo "  stop     - 컨테이너 중지"
        exit 1
        ;;
esac
