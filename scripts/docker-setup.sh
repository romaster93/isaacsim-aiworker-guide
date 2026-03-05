#!/bin/bash
# IsaacSim Docker 환경 사전 설정 스크립트
# 호스트에서 한 번만 실행하면 됩니다.

set -e

echo "=== IsaacSim Docker 사전 설정 ==="
echo ""

# 1. NVIDIA Container Toolkit 설치 확인
echo "[1/4] NVIDIA Container Toolkit 확인..."
if command -v nvidia-ctk &> /dev/null; then
    echo "  -> 이미 설치됨: $(nvidia-ctk --version 2>/dev/null || echo 'installed')"
else
    echo "  -> 설치 중..."
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
        sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    echo "  -> 설치 완료"
fi

# 2. NVIDIA Driver 확인
echo ""
echo "[2/4] NVIDIA Driver 확인..."
if command -v nvidia-smi &> /dev/null; then
    DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
    echo "  -> Driver: $DRIVER_VERSION"
    echo "  -> GPU:"
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | sed 's/^/     /'
else
    echo "  [ERROR] nvidia-smi를 찾을 수 없습니다. NVIDIA Driver를 먼저 설치하세요."
    exit 1
fi

# 3. Docker 확인
echo ""
echo "[3/4] Docker 확인..."
if command -v docker &> /dev/null; then
    echo "  -> Docker: $(docker --version)"
    # Docker 권한 확인
    if ! docker info &> /dev/null; then
        echo "  [WARN] Docker 권한 없음. 다음 명령 실행 후 재로그인:"
        echo "         sudo usermod -aG docker \$USER"
    fi
else
    echo "  [ERROR] Docker가 설치되어 있지 않습니다."
    exit 1
fi

# 4. NGC 로그인 확인
echo ""
echo "[4/4] NGC 이미지 접근 확인..."
if docker image inspect nvcr.io/nvidia/isaac-sim:5.1.0 &> /dev/null; then
    echo "  -> 이미지 이미 존재 (pull 불필요)"
else
    echo "  -> 이미지가 없습니다. pull이 필요합니다."
    echo "  -> NGC API Key가 필요합니다: https://org.ngc.nvidia.com/setup"
    echo ""
    read -p "  NGC 로그인 후 pull하시겠습니까? (y/N): " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        echo "  Docker 로그인 (Username: \$oauthtoken, Password: NGC API Key):"
        docker login nvcr.io
        echo "  이미지 pulling... (약 9.4GB, 시간이 걸립니다)"
        docker pull nvcr.io/nvidia/isaac-sim:5.1.0
    else
        echo "  -> 나중에 수동으로 pull하세요:"
        echo "     docker login nvcr.io"
        echo "     docker pull nvcr.io/nvidia/isaac-sim:5.1.0"
    fi
fi

echo ""
echo "=== 설정 완료 ==="
echo ""
echo "다음 단계:"
echo "  cd $(dirname "$0")/.."
echo "  docker compose up -d"
echo "  docker exec -it isaac-sim bash"
