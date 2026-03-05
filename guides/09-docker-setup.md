# 번외: Docker로 IsaacSim 실행하기

## Overview

Docker를 사용하면 IsaacSim을 직접 설치하지 않고도 컨테이너로 실행할 수 있습니다.
다른 컴퓨터에 IsaacSim, ROS2, CUDA를 따로 설치할 필요 없이, NVIDIA Driver만 있으면 동일한 환경을 바로 재현할 수 있습니다.

> **핵심**: NVIDIA 공식 이미지(`nvcr.io/nvidia/isaac-sim:5.1.0`)에 IsaacSim + ROS2 Bridge + CUDA가 **모두 포함**되어 있습니다.
> 호스트에는 NVIDIA Driver와 Docker만 설치하면 됩니다.

### Docker 실행 구조

```
호스트 (Ubuntu 22.04 또는 24.04)
├── NVIDIA Driver (호스트에 설치 필수)
├── Docker + NVIDIA Container Toolkit
└── IsaacSim 컨테이너 (nvcr.io/nvidia/isaac-sim:5.1.0)
    ├── Ubuntu 22.04 (컨테이너 내부 OS)
    ├── CUDA 12.x
    ├── IsaacSim 5.1.0
    ├── ROS2 Bridge (내장)
    └── /isaac-sim/workspace ← 볼륨 마운트 (호스트의 USD 파일)
```

> **호스트 OS가 24.04여도 됩니다.**
> Docker 컨테이너는 호스트 커널을 공유하고, 유저스페이스(라이브러리 등)는 독립적이기 때문에
> 호스트 Ubuntu 24.04 위에서 컨테이너 내부 Ubuntu 22.04가 문제없이 동작합니다.

## Prerequisites

### 필수 요구사항

| 항목 | 최소 요구 | 확인 방법 |
|------|----------|----------|
| **NVIDIA GPU** | RTX 2070 이상 (VRAM 8GB+) | `nvidia-smi` |
| **NVIDIA Driver** | 535.129+ (권장: 570+) | `nvidia-smi` 상단 |
| **Linux Kernel** | 5.4+ | `uname -r` |
| **Docker** | 24.0+ | `docker --version` |
| **NVIDIA Container Toolkit** | 1.14+ | `nvidia-ctk --version` |
| **디스크 공간** | 약 25GB (이미지 ~10GB + 캐시 ~15GB) | `df -h` |
| **RAM** | 32GB+ (권장: 64GB) | `free -h` |

### 호스트 OS 호환성

| 호스트 OS | 지원 여부 | 비고 |
|-----------|----------|------|
| Ubuntu 22.04 | O | 완전 지원 |
| Ubuntu 24.04 | O | 완전 지원 (커널 공유 방식) |
| Ubuntu 20.04 | △ | 커널 버전 확인 필요 |
| CentOS/RHEL 8+ | O | Driver 설치 방법 다름 |
| Windows (WSL2) | △ | GPU 패스스루 제한 있음 |

---

## [1] NVIDIA Driver 설치/확인

### 이미 설치된 경우

```bash
nvidia-smi
```

출력 예시:
```
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 570.211.01    Driver Version: 570.211.01    CUDA Version: 12.8               |
|   GPU  Name         Persistence-M | ...
|   0    NVIDIA RTX PRO 6000   On   | ...   98304MiB |
+-----------------------------------------------------------------------------------------+
```

> **Driver 버전 535 이상**이면 IsaacSim 5.1.0 호환됩니다.
> CUDA Version은 **호스트에 CUDA를 따로 설치할 필요 없습니다** — 컨테이너 안에 포함되어 있습니다.

### 설치가 필요한 경우

```bash
# Ubuntu 22.04/24.04 권장 방법
sudo apt update
sudo apt install -y nvidia-driver-570
sudo reboot
```

> **주의**: 커널 업데이트 후 Driver가 깨질 수 있습니다.
> `sudo apt install -y dkms`로 DKMS를 설치해두면 커널 업데이트 시 자동 재빌드됩니다.

### NVIDIA Driver와 CUDA의 관계

```
호스트: NVIDIA Driver 570 ← GPU 하드웨어와 직접 통신
         └── CUDA 호환: 12.x까지 지원 (하위 호환)

컨테이너: CUDA 12.x ← 컨테이너 안에 설치됨
          └── 호스트 Driver를 통해 GPU 접근
```

- **호스트에는 CUDA 설치 불필요** — Driver만 있으면 됨
- 컨테이너가 사용하는 CUDA 버전은 호스트 Driver가 지원하는 범위 내여야 함
- Driver 535+ → CUDA 12.2까지, Driver 570+ → CUDA 12.8까지 지원

---

## [2] Docker 설치

### Docker Engine 설치

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

### 설치 확인

```bash
docker --version     # Docker version 27.x.x 이상
docker compose version  # Docker Compose v2.x.x 이상
```

---

## [3] NVIDIA Container Toolkit 설치

Docker 컨테이너에서 GPU를 사용하려면 NVIDIA Container Toolkit이 필요합니다.

### Container Toolkit이란?

```
일반 Docker 컨테이너:
  [컨테이너] ──X──> [GPU]  (접근 불가)

NVIDIA Container Toolkit 설치 후:
  [컨테이너] ──→ [nvidia-container-runtime] ──→ [GPU]  (접근 가능)
```

Docker가 컨테이너를 실행할 때 자동으로 GPU 디바이스(`/dev/nvidia*`)와 드라이버 라이브러리를 마운트해줍니다.

### 설치

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
# GPU가 컨테이너에서 보이는지 확인
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

호스트와 동일한 `nvidia-smi` 출력이 나오면 정상입니다.

---

## [4] IsaacSim 이미지 Pull

### NGC 계정 준비

NVIDIA NGC(GPU Cloud)에서 이미지를 받으려면 API Key가 필요합니다.

1. https://org.ngc.nvidia.com/setup 접속
2. 계정 생성/로그인
3. **API Key** 생성 → 복사 (한 번만 표시되므로 저장해둘 것)

### Docker 로그인 & Pull

```bash
# NGC 레지스트리 로그인
docker login nvcr.io
# Username: $oauthtoken  (그대로 입력)
# Password: <NGC API Key 붙여넣기>

# 이미지 Pull (~9.4GB, 네트워크에 따라 10~30분)
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

### Pull 확인

```bash
docker images | grep isaac-sim
# nvcr.io/nvidia/isaac-sim   5.1.0   xxxxxxxxxxxx   9.4GB
```

---

## [5] 프로젝트 파일 준비

### 프로젝트 구조

```
ms_AIworker/
├── docker-compose.yml          ← 컨테이너 설정
├── scripts/
│   ├── docker-setup.sh         ← 사전 설정 자동화
│   └── docker-run.sh           ← 실행 스크립트
└── isaacsim_ai_worker/         ← 컨테이너에 마운트됨
    └── usd_ai_worker/
        └── Collected_World2/
            └── World2.usd      ← 센서 설정 완료된 Stage
```

### 다른 컴퓨터로 옮기기

```bash
# 방법 1: Git clone (USD 파일이 Git에 포함된 경우)
git clone https://github.com/romaster93/isaacsim-aiworker-guide.git
cd isaacsim-aiworker-guide

# 방법 2: 폴더 통째로 복사
scp -r ms_AIworker/ user@other-pc:/home/user/
```

> **Collected_World2/ 폴더** (~570MB)가 핵심입니다.
> `Collect As`로 저장했기 때문에 모든 에셋이 상대 경로로 포함되어 있어 그대로 동작합니다.

---

## [6] 실행

### 방법 1: docker compose (권장)

```bash
cd ms_AIworker

# 컨테이너 시작
docker compose up -d

# 컨테이너 쉘 접속
docker exec -it isaac-sim bash
```

컨테이너 안에서:
```bash
# GUI 모드 (호스트에 모니터 연결 시)
./runapp.sh

# Headless 모드 (Python 스크립트 전용)
./runheadless.native.sh -v

# Livestream 모드 (WebRTC 원격 접속)
./runheadless.sh -v
```

### 방법 2: 실행 스크립트

```bash
# 사전 설정 (최초 1회)
chmod +x scripts/docker-setup.sh scripts/docker-run.sh
./scripts/docker-setup.sh

# GUI 모드
./scripts/docker-run.sh gui

# Headless 모드
./scripts/docker-run.sh headless

# Livestream 모드
./scripts/docker-run.sh stream

# 쉘만 접속
./scripts/docker-run.sh shell

# 중지
./scripts/docker-run.sh stop
```

### 방법 3: docker run (직접 실행)

```bash
# GUI 모드
xhost +local:
docker run --name isaac-sim --entrypoint bash -it \
  --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -e "DISPLAY=$DISPLAY" \
  --rm \
  --network=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/isaacsim_ai_worker:/isaac-sim/workspace:rw \
  nvcr.io/nvidia/isaac-sim:5.1.0
```

---

## [7] 컨테이너 안에서 USD 파일 열기

컨테이너에 접속 후 IsaacSim이 실행되면:

1. Content 브라우저에서 `/isaac-sim/workspace/` 경로 탐색
2. `usd_ai_worker/Collected_World2/World2.usd` 더블클릭
3. 기존에 설정한 센서, Action Graph, 환경이 모두 그대로 로드됨

### ROS2 토픽 확인

**호스트에서 확인** (권장):
```bash
# network_mode: host이므로 호스트에서 바로 토픽 확인 가능
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

> **`network_mode: host`** 를 사용하기 때문에 컨테이너와 호스트가 같은 네트워크를 공유합니다.
> 호스트에서 `ros2 topic list`로 컨테이너 안의 IsaacSim 토픽을 바로 볼 수 있습니다.
> RViz2도 호스트에서 실행하여 시각화할 수 있습니다.

> **주의**: 컨테이너 안에는 `/opt/ros/humble`이 설치되어 있지 않습니다.
> ROS2 CLI(`ros2 topic list` 등)는 **호스트에서** 실행하세요.
> 컨테이너 내부에는 IsaacSim의 ROS2 Bridge 내장 라이브러리만 포함되어 있어
> IsaacSim이 토픽을 발행하는 것은 가능하지만, ROS2 CLI 도구는 없습니다.

---

## [8] 실행 모드 비교

| 모드 | 명령어 | GUI | 용도 | 접속 방법 |
|------|--------|-----|------|----------|
| **GUI** | `./runapp.sh` | 모니터에 직접 표시 | 개발/디버깅 | 로컬 모니터 |
| **Headless** | `./runheadless.native.sh` | 없음 | Python 스크립트 자동화 | 터미널만 |
| **Livestream** | `./runheadless.sh` | WebRTC 원격 | 원격 서버 작업 | 웹브라우저 |

### Livestream 접속 방법

Livestream 모드 실행 후:
1. NVIDIA Isaac Sim WebRTC Streaming Client 다운로드
   - https://docs.isaacsim.omniverse.nvidia.com/ → Downloads
2. 클라이언트 실행 → Server IP: `<호스트 IP>` 입력
3. IsaacSim GUI가 원격으로 표시됨

> **서버(SSH 접속만 가능한 PC)에서 작업할 때** Livestream 모드가 유용합니다.

---

## docker-compose.yml 설명

```yaml
services:
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:5.1.0   # NVIDIA 공식 이미지
    network_mode: host       # 호스트와 네트워크 공유 (ROS2 DDS 통신)
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all     # 모든 GPU 사용 (단일 GPU: count: 1)
              capabilities: [gpu]
    environment:
      - ACCEPT_EULA=Y        # 라이선스 동의 (필수)
      - PRIVACY_CONSENT=Y    # 개인정보 동의 (필수)
      - ROS_DISTRO=humble    # 내장 ROS2 배포판
      - DISPLAY=${DISPLAY}   # GUI 모드용 X11 디스플레이
    volumes:
      # 프로젝트 파일 마운트
      - ./isaacsim_ai_worker:/isaac-sim/workspace:rw
      # X11 소켓 (GUI 모드)
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    user: "1234:1234"        # 컨테이너 내부 사용자 (rootless)
```

### volume 설명

| 마운트 | 용도 |
|--------|------|
| `isaac-cache` | Shader 캐시 등. 재시작 시 로딩 속도 향상 |
| `isaac-computecache` | GPU 연산 캐시 |
| `isaac-logs` | IsaacSim 로그 파일 |
| `isaac-config` | Omniverse 설정 |
| `./isaacsim_ai_worker` | **프로젝트 USD 파일** (호스트 ↔ 컨테이너 실시간 공유) |
| `/tmp/.X11-unix` | X11 소켓 (GUI 모드에서 모니터 출력) |

---

## Troubleshooting

### docker run 시 GPU를 못 찾음
```
docker: Error response from daemon: could not select device driver ""
```
- NVIDIA Container Toolkit이 설치 안 됨 → [3] 단계 수행
- 설치 후 `sudo systemctl restart docker` 필수

### 컨테이너 안에서 nvidia-smi 안 됨
```
Failed to initialize NVML: Unknown Error
```
- `--gpus all` 플래그 확인
- 호스트에서 `nvidia-smi`가 되는지 먼저 확인
- Driver 버전이 535 미만이면 업데이트 필요

### GUI 모드에서 화면이 안 나옴
```
cannot open display :0
```
1. 호스트에서 `xhost +local:` 실행
2. `DISPLAY` 환경변수 확인: `echo $DISPLAY`
3. SSH 접속인 경우 GUI 불가 → Livestream 모드 사용

### Permission denied (볼륨 마운트)
- 컨테이너가 UID 1234로 실행됨
- 호스트 파일 권한이 맞지 않으면: `chmod -R a+rw isaacsim_ai_worker/`

### ROS2 토픽이 호스트에서 안 보임
- `docker-compose.yml`에 `network_mode: host` 확인
- 호스트와 컨테이너의 `RMW_IMPLEMENTATION`이 일치해야 함
  - 기본: `rmw_fastrtps_cpp` (양쪽 동일하면 OK)
- 방화벽 확인: `sudo ufw status` → DDS 포트 열려있는지

### 컨테이너 내부 ROS2 버전
- 공식 이미지(5.1.0)는 **Ubuntu 22.04 기반**이라 **ROS2 Humble** 내장
- 호스트가 **ROS2 Jazzy** (Ubuntu 24.04)여도 `network_mode: host`면 DDS 통신 호환됨
- Humble ↔ Jazzy 간 메시지 호환은 대부분 OK (표준 메시지 타입)

### 이미지 Pull 속도가 너무 느림
- 이미지 크기가 ~9.4GB로 큼
- 안정적인 유선 네트워크 권장
- Pull 중단 시 `docker pull`을 다시 실행하면 이어받기됨

### 커널 업데이트 후 GPU 인식 안 됨
```
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver
```
- 커널이 업데이트되면 NVIDIA 커널 모듈도 재빌드 필요
- `sudo apt install -y dkms nvidia-driver-570` 후 `sudo reboot`
- 또는: `sudo dkms autoinstall` 후 `sudo reboot`

---

## 요약: 다른 PC에서 재현하는 전체 순서

```bash
# 1. NVIDIA Driver 설치 (535+)
sudo apt install -y nvidia-driver-570
sudo reboot

# 2. Docker 설치
# (위 [2] 참고)

# 3. NVIDIA Container Toolkit 설치
# (위 [3] 참고)

# 4. 프로젝트 복사
git clone https://github.com/romaster93/isaacsim-aiworker-guide.git
cd isaacsim-aiworker-guide

# 5. NGC 로그인 & 이미지 Pull
docker login nvcr.io
docker pull nvcr.io/nvidia/isaac-sim:5.1.0

# 6. 실행
docker compose up -d
docker exec -it isaac-sim bash
./runapp.sh

# 7. USD 파일 열기
# Content 브라우저 → /isaac-sim/workspace/usd_ai_worker/Collected_World2/World2.usd
```

---
**Status**: COMPLETED
**생성된 파일**: `docker-compose.yml`, `scripts/docker-setup.sh`, `scripts/docker-run.sh`
**관련 가이드**: 이 문서는 번외 가이드입니다. 메인 순서는 [Step 1](01-install-isaacsim.md)부터 시작하세요.
