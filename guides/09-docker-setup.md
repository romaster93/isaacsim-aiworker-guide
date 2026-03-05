# 번외: Docker로 IsaacSim 실행하기

## Overview

Docker를 사용하면 IsaacSim을 직접 설치하지 않고도 컨테이너로 실행할 수 있습니다.
다른 컴퓨터에 IsaacSim, ROS2, CUDA를 따로 설치할 필요 없이, NVIDIA Driver만 있으면 동일한 환경을 바로 재현할 수 있습니다.

> **핵심**: NVIDIA 공식 이미지 기반으로 ROS2 Jazzy CLI를 추가한 커스텀 이미지를 빌드합니다.
> 호스트에는 NVIDIA Driver와 Docker만 설치하면 됩니다.

### Docker 실행 구조

```
호스트 (Ubuntu 22.04 또는 24.04)
├── NVIDIA Driver (호스트에 설치 필수)
├── Docker + NVIDIA Container Toolkit
└── IsaacSim 컨테이너 (커스텀 이미지)
    ├── Ubuntu 24.04 (컨테이너 내부 OS)
    ├── CUDA 12.x
    ├── IsaacSim 5.1.0
    ├── ROS2 Jazzy CLI (ros2 명령어 사용 가능)
    ├── ROS2 Bridge (IsaacSim 내장, Humble)
    ├── FastDDS UDP 설정 (호스트↔컨테이너 통신)
    └── /isaac-sim/workspace ← 볼륨 마운트 (호스트의 USD 파일)
```

### 핵심 구성 파일

| 파일 | 역할 |
|------|------|
| `Dockerfile` | IsaacSim 기반 + ROS2 Jazzy CLI 설치 + FastDDS 설정 |
| `docker-compose.yml` | 컨테이너 실행 설정 (GPU, 볼륨, X11) |
| `fastdds.xml` | DDS를 UDP로 강제 (Shared Memory 비활성화) |
| `scripts/docker-run.sh` | 원클릭 실행 스크립트 |
| `scripts/ros2-docker.sh` | 호스트에서 컨테이너 토픽 확인 (호스트 환경 변경 없음) |

> **왜 FastDDS UDP가 필요한가?**
> FastDDS는 기본적으로 Shared Memory 통신을 사용합니다. 하지만 Docker 컨테이너와 호스트 간에는
> Shared Memory가 공유되지 않아 토픽 데이터가 전달되지 않습니다.
> `fastdds.xml`로 UDP 통신을 강제하면 `network_mode: host`를 통해 정상 통신됩니다.

---

## Prerequisites

### 필수 요구사항

| 항목 | 최소 요구 | 확인 방법 |
|------|----------|----------|
| **NVIDIA GPU** | RTX 2070 이상 (VRAM 8GB+) | `nvidia-smi` |
| **NVIDIA Driver** | 535.129+ (권장: 570+) | `nvidia-smi` 상단 |
| **Linux Kernel** | 5.4+ | `uname -r` |
| **Docker** | 24.0+ | `docker --version` |
| **NVIDIA Container Toolkit** | 1.14+ | `nvidia-ctk --version` |
| **디스크 공간** | 약 30GB (이미지 ~15GB + 캐시 ~15GB) | `df -h` |
| **RAM** | 32GB+ (권장: 64GB) | `free -h` |

### 호스트 OS 호환성

| 호스트 OS | 지원 여부 | 비고 |
|-----------|----------|------|
| Ubuntu 22.04 | O | 완전 지원 |
| Ubuntu 24.04 | O | 완전 지원 |
| Ubuntu 20.04 | △ | 커널 버전 확인 필요 |
| CentOS/RHEL 8+ | O | Driver 설치 방법 다름 |

---

## [1] NVIDIA Driver 설치/확인

### 이미 설치된 경우

```bash
nvidia-smi
```

> **Driver 버전 535 이상**이면 IsaacSim 5.1.0 호환됩니다.
> CUDA는 호스트에 따로 설치할 필요 없습니다 — 컨테이너 안에 포함되어 있습니다.

### 설치가 필요한 경우

```bash
sudo apt update
sudo apt install -y nvidia-driver-570
sudo reboot
```

---

## [2] Docker 설치

```bash
# 기존 패키지 제거
sudo apt remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true

# Docker 공식 GPG 키 추가
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# 레포지토리 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 설치
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 현재 사용자를 docker 그룹에 추가 (sudo 없이 docker 사용)
sudo usermod -aG docker $USER
```

> **중요**: `usermod` 후 **재로그인** (또는 `newgrp docker`)해야 반영됩니다.

---

## [3] NVIDIA Container Toolkit 설치

Docker 컨테이너에서 GPU를 사용하려면 NVIDIA Container Toolkit이 필요합니다.

```bash
# GPG 키 추가
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# 레포지토리 추가
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 설치
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Docker 런타임 설정
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 설치 확인

```bash
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

---

## [4] NGC 로그인 & 베이스 이미지 Pull

NVIDIA NGC에서 IsaacSim 이미지를 받으려면 API Key가 필요합니다.

1. https://org.ngc.nvidia.com/setup 접속
2. 계정 생성/로그인
3. **API Key** 생성 → 복사

```bash
# NGC 레지스트리 로그인
docker login nvcr.io
# Username: $oauthtoken  (그대로 입력)
# Password: <NGC API Key 붙여넣기>

# 베이스 이미지 Pull (~9.4GB)
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

---

## [5] 프로젝트 클론 & 이미지 빌드

```bash
git clone https://github.com/romaster93/isaacsim-aiworker-guide.git
cd isaacsim-aiworker-guide

# 커스텀 이미지 빌드 (ROS2 Jazzy CLI + FastDDS 설정 추가)
docker compose build
```

빌드 완료 후 이미지 확인:
```bash
docker images | grep isaac-sim-ros2
# isaac-sim-ros2   5.1.0   xxxxxxxxxxxx   ~15GB
```

### Dockerfile 내용 요약

```dockerfile
FROM nvcr.io/nvidia/isaac-sim:5.1.0

# ROS2 Jazzy CLI 설치 (Ubuntu 24.04 Noble 기반)
# → 컨테이너 안에서 ros2 topic list 등 CLI 명령어 사용 가능

# FastDDS XML 설정 복사
# → Shared Memory 대신 UDP 사용 (호스트↔컨테이너 통신)

# 볼륨 디렉토리 권한 설정
# → Named Volume 권한 문제 방지
```

---

## [6] 실행

### 방법 1: 실행 스크립트 (권장)

```bash
# 사전 설정 (최초 1회)
chmod +x scripts/docker-run.sh scripts/ros2-docker.sh

# GUI 모드로 실행 (자동으로 빌드, 권한설정, 컨테이너 시작)
./scripts/docker-run.sh gui
```

컨테이너 쉘에 접속되면:
```bash
./runapp.sh
```

호스트에서 ROS2 토픽 확인 (호스트 환경 변경 없음):
```bash
./scripts/ros2-docker.sh topic list
./scripts/ros2-docker.sh topic echo /tf --once
```

실행 스크립트 모드:
| 명령어 | 설명 |
|--------|------|
| `./scripts/docker-run.sh gui` | GUI 모드 (모니터에 직접 표시) |
| `./scripts/docker-run.sh headless` | Headless 모드 (Python 스크립트 전용) |
| `./scripts/docker-run.sh stream` | Livestream 모드 (WebRTC 원격) |
| `./scripts/docker-run.sh shell` | 컨테이너 쉘만 접속 |
| `./scripts/docker-run.sh ros2` | 호스트 ROS2 설정 안내 |
| `./scripts/docker-run.sh stop` | 컨테이너 중지 |

### 방법 2: 직접 실행

```bash
# 파일 권한 설정
chmod -R o+rX isaacsim_ai_worker/

# X11 허용
xhost +local:

# 컨테이너 시작
docker compose up -d

# 볼륨 권한 수정 (최초 1회)
docker exec -u root isaac-sim chown -R isaac-sim:isaac-sim \
    /isaac-sim/.cache /isaac-sim/.local /isaac-sim/.nvidia-omniverse /isaac-sim/.nv

# 접속 & 실행
docker exec -it isaac-sim bash
./runapp.sh
```

---

## [7] 컨테이너 안에서 USD 파일 열기

IsaacSim GUI가 실행되면:

1. Content 브라우저에서 `/isaac-sim/workspace/` 경로 탐색
2. `usd_ai_worker/Collected_World2/World2.usd` 더블클릭
3. 기존에 설정한 센서, Action Graph, 환경이 모두 그대로 로드됨
4. **Play** 버튼 클릭하여 시뮬레이션 시작

### ROS2 토픽 확인

Docker 컨테이너의 IsaacSim이 발행하는 토픽을 호스트에서 확인하려면 **FastDDS UDP 설정**이 필요합니다.
(기본 Shared Memory 통신은 컨테이너↔호스트 간에 동작하지 않음)

#### 방법 A: conda 환경 사용 (권장 — 호스트에서 ros2, rviz2 그대로 사용)

자동 설정 스크립트를 실행하면 호스트의 ROS2 버전(Humble/Jazzy)을 감지하고 conda 환경에 FastDDS 설정을 추가합니다:
```bash
# 최초 1회 실행
./scripts/setup-host-ros2.sh
```

이후 conda 환경만 활성화하면 됩니다:
```bash
conda activate isaac_sim
ros2 topic list
ros2 topic echo /tf --once
rviz2  # RViz도 그냥 사용 가능
```

> **원리**: `conda activate isaac_sim` 시 `FASTRTPS_DEFAULT_PROFILES_FILE`과 `RMW_IMPLEMENTATION`이
> 자동 설정됩니다. `conda deactivate` 하면 원래대로 복원되어 호스트 환경에 영향 없습니다.

> **호스트 ROS2 버전별 차이**:
> | 호스트 ROS2 | Docker 내부 Bridge | DDS 호환 | 비고 |
> |------------|-------------------|---------|------|
> | **Jazzy** (Ubuntu 24.04) | Humble Bridge | O | DDS 프로토콜 레벨 호환 |
> | **Humble** (Ubuntu 22.04) | Humble Bridge | O | 동일 버전, 가장 안정적 |
>
> 두 경우 모두 `fastdds.xml` (UDP 강제)이 필요합니다.

#### 방법 B: ros2-docker.sh 래퍼 스크립트 (호스트 환경 변경 없이)

```bash
./scripts/ros2-docker.sh topic list
./scripts/ros2-docker.sh topic echo /tf --once
./scripts/ros2-docker.sh topic hz /tf
```

> `ros2-docker.sh`는 FastDDS UDP 설정과 `RMW_IMPLEMENTATION`을 **임시로** 적용하여 `ros2` 명령을 실행합니다. 호스트의 환경변수는 변경하지 않습니다. 단, `rviz2`는 이 방법으로 실행할 수 없습니다.

#### 방법 C: 컨테이너 안에서 직접 확인

```bash
# 별도 터미널에서 컨테이너 접속
docker exec -it isaac-sim bash
ros2 topic list
ros2 topic echo /tf --once
```

---

## [8] 실행 모드 비교

| 모드 | 명령어 | GUI | 용도 | 접속 방법 |
|------|--------|-----|------|----------|
| **GUI** | `./runapp.sh` | 모니터에 직접 표시 | 개발/디버깅 | 로컬 모니터 |
| **Headless** | `./runheadless.native.sh` | 없음 | Python 스크립트 자동화 | 터미널만 |
| **Livestream** | `./runheadless.sh` | WebRTC 원격 | 원격 서버 작업 | 웹브라우저 |

> **서버(SSH 접속만 가능한 PC)에서 작업할 때** Livestream 모드가 유용합니다.

---

## docker-compose.yml 설명

```yaml
services:
  isaac-sim:
    build: .                     # Dockerfile로 커스텀 이미지 빌드
    image: isaac-sim-ros2:5.1.0  # 빌드된 이미지 이름
    network_mode: host           # 호스트와 네트워크 공유 (ROS2 DDS 통신)
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all         # 모든 GPU 사용
              capabilities: [gpu]
    environment:
      - ACCEPT_EULA=Y            # 라이선스 동의 (필수)
      - PRIVACY_CONSENT=Y        # 개인정보 동의 (필수)
      - DISPLAY=${DISPLAY}       # GUI 모드용 X11
    volumes:
      - ./isaacsim_ai_worker:/isaac-sim/workspace:rw  # USD 파일 마운트
      - /tmp/.X11-unix:/tmp/.X11-unix:rw              # X11 소켓
```

### Volume 설명

| 마운트 | 용도 |
|--------|------|
| `isaac-cache` | Shader 캐시. 재시작 시 로딩 속도 향상 |
| `isaac-computecache` | GPU 연산 캐시 |
| `isaac-logs` | IsaacSim 로그 파일 |
| `isaac-config` | Omniverse 설정 |
| `./isaacsim_ai_worker` | **프로젝트 USD 파일** (호스트 ↔ 컨테이너 실시간 공유) |

---

## Troubleshooting

### GPU를 못 찾음
```
docker: Error response from daemon: could not select device driver ""
```
→ NVIDIA Container Toolkit 미설치. [3] 단계 수행 후 `sudo systemctl restart docker`

### GUI 화면이 안 나옴
```
cannot open display :0
```
1. 호스트에서 `xhost +local:` 실행
2. `echo $DISPLAY` 확인
3. SSH 접속인 경우 → Livestream 모드 사용

### 텍스처가 빨간색 (Permission denied)
컨테이너의 `isaac-sim` 유저가 호스트 파일을 못 읽는 경우:
```bash
# 호스트 파일 권한 열기
chmod -R o+rX isaacsim_ai_worker/

# 컨테이너 볼륨 권한 수정
docker exec -u root isaac-sim chown -R isaac-sim:isaac-sim \
    /isaac-sim/.cache /isaac-sim/.local /isaac-sim/.nvidia-omniverse /isaac-sim/.nv
```

### ROS2 Bridge startup failed
```
failed to load shared library 'librmw_fastrtps_cpp.so'
```
→ IsaacSim 내부 `setup_ros_env.sh`가 자동으로 환경 설정합니다. `./runapp.sh`로 실행하면 자동 처리됨.

### 호스트에서 ROS2 토픽 데이터가 안 옴
FastDDS Shared Memory 문제입니다. `ros2-docker.sh`를 사용하세요:
```bash
./scripts/ros2-docker.sh topic list        # 토픽 목록
./scripts/ros2-docker.sh topic echo /tf    # TF 데이터
```

### 컨테이너 이름 충돌 (container name already in use)
```
Error response from daemon: Conflict. The container name "/isaac-sim" is already in use
```
→ 이전에 실행했던 컨테이너가 남아있는 경우입니다. 기존 컨테이너를 정리하고 다시 실행하세요:
```bash
docker stop isaac-sim && docker rm isaac-sim
./scripts/docker-run.sh gui
```
> **참고**: `docker compose down`으로도 정리 가능하지만, 이 경우 Named Volume은 유지됩니다 (캐시 보존).
> 볼륨까지 완전히 삭제하려면 `docker compose down -v`를 사용하세요.

### 커널 업데이트 후 GPU 인식 안 됨
```
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver
```
→ `sudo apt install -y dkms nvidia-driver-570` 후 `sudo reboot`

---

## 요약: 다른 PC에서 재현하는 전체 순서

```bash
# 1. NVIDIA Driver 설치 (535+)
sudo apt install -y nvidia-driver-570 && sudo reboot

# 2. Docker 설치 (위 [2] 참고)

# 3. NVIDIA Container Toolkit 설치 (위 [3] 참고)

# 4. 프로젝트 클론
git clone https://github.com/romaster93/isaacsim-aiworker-guide.git
cd isaacsim-aiworker-guide

# 5. NGC 로그인 & 베이스 이미지 Pull
docker login nvcr.io    # Username: $oauthtoken, Password: NGC API Key
docker pull nvcr.io/nvidia/isaac-sim:5.1.0

# 6. 커스텀 이미지 빌드
docker compose build

# 7. 실행
chmod +x scripts/docker-run.sh scripts/ros2-docker.sh
./scripts/docker-run.sh gui
# 컨테이너 안에서: ./runapp.sh

# 8. USD 파일 열기
# Content 브라우저 → /isaac-sim/workspace/usd_ai_worker/Collected_World2/World2.usd

# 9. 호스트에서 토픽 확인
# 방법 A: conda 환경 (rviz2도 사용 가능)
conda activate isaac_sim
ros2 topic list
ros2 topic echo /tf --once
# 방법 B: 래퍼 스크립트 (환경 변경 없음)
./scripts/ros2-docker.sh topic list
```

---
**Status**: COMPLETED
**생성된 파일**: `Dockerfile`, `docker-compose.yml`, `fastdds.xml`, `scripts/docker-run.sh`, `scripts/ros2-docker.sh`
**관련 가이드**: 이 문서는 번외 가이드입니다. 메인 순서는 [Step 1](01-install-isaacsim.md)부터 시작하세요.
