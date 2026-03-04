# Step 1: Install IsaacSim 5.1.0

## Overview
IsaacSim 5.1.0을 conda 환경에서 pip로 설치합니다.
- 5.1.0은 Ubuntu 24.04를 공식 지원합니다.
- Python 3.11이 필요합니다.

## Prerequisites
- [x] Ubuntu 24.04
- [x] NVIDIA GPU with RT Cores (RTX PRO 6000 Blackwell)
- [x] NVIDIA Driver 570+ (installed: 570.211.01)
- [x] conda (installed: 25.11.1)

## Step 1.1: Conda 환경 생성

```bash
conda create -n isaac_sim python=3.11 -y
```

## Step 1.2: 환경 활성화

```bash
conda activate isaac_sim
```

## Step 1.3: IsaacSim 5.1.0 설치

```bash
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

> **Note**: 설치에 상당한 시간이 걸릴 수 있습니다 (10-30분).

## Step 1.4: EULA 동의 및 첫 실행

### 기본 실행 (ROS2 불필요 시)
```bash
conda activate isaac_sim
unset LD_PRELOAD
unset LD_LIBRARY_PATH
isaacsim
```

### ROS2 연동 실행 (센서 데이터 발행 등)

**방법 1: 수동 환경변수 설정 (매번 실행 시)**
```bash
conda activate isaac_sim
unset LD_PRELOAD
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="/home/cho/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/exts/3/isaacsim.ros2.bridge-4.12.4+107.3.3.lx64/jazzy/lib"
isaacsim
```

**방법 2: conda 자동 환경변수 설정 (권장, 1회만 설정)**

매번 환경변수를 수동 설정하는 대신, conda activate/deactivate 시 자동으로 설정되도록 스크립트를 추가합니다:

```bash
# activate 스크립트 생성
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
cat > $CONDA_PREFIX/etc/conda/activate.d/isaacsim_env.sh << 'EOF'
#!/bin/bash
unset LD_PRELOAD
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="/home/cho/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/exts/3/isaacsim.ros2.bridge-4.12.4+107.3.3.lx64/jazzy/lib"
EOF

# deactivate 스크립트 생성
mkdir -p $CONDA_PREFIX/etc/conda/deactivate.d
cat > $CONDA_PREFIX/etc/conda/deactivate.d/isaacsim_env.sh << 'EOF'
#!/bin/bash
unset ROS_DISTRO
unset RMW_IMPLEMENTATION
unset LD_LIBRARY_PATH
EOF
```

설정 후에는 아래 명령어만으로 ROS2 연동 실행 가능:
```bash
conda activate isaac_sim
isaacsim
```

> **중요**: ROS2 bridge를 사용하려면 반드시 위 환경변수를 설정해야 합니다.
> `LD_LIBRARY_PATH`를 unset하면 ROS2 bridge가 `libament_index_cpp.so`를 찾지 못해 실패합니다.

- "Do you accept EULA?" 나오면 → `yes` 입력
- 최초 실행 시 상당한 시간 대기 필요 (셰이더 컴파일 등)
- "Isaac Sim Full is not responding" → **Wait** 클릭

> **주의**: `LD_PRELOAD`나 `LD_LIBRARY_PATH`가 설정되어 있으면 glibc TLS 에러가 발생할 수 있습니다:
> `Inconsistency detected by ld.so: ../elf/dl-tls.c: 613: _dl_allocate_tls_init: Assertion 'listp != NULL' failed!`
> 반드시 `unset`으로 정리 후 실행하세요.

## Step 1.5: 첫 실행 후 설정

### Grid 표시 (기본값이 꺼져있음!)
첫 실행 시 Viewport가 **검은 화면**일 수 있습니다. Grid가 기본적으로 비활성화되어 있기 때문입니다:
1. Viewport 상단의 **눈 모양 아이콘 (Display)** 클릭
2. **Grid** 항목 체크
3. Grid 선이 Viewport에 표시되면 정상

### Renderer 확인
Grid를 켜도 검은 화면이면 렌더러를 확인하세요:
1. `Edit` → `Preferences` → `Rendering`
2. **RTX - Real-Time** 체크 (기본 작업용, 가장 빠름)
3. 옵션: Real-Time 2.0, Real-Time, Interactive (Path Tracing)

## Step 1.6: 설치 확인

아래 항목이 모두 보이면 성공:
- [x] IsaacSim GUI 창이 뜸
- [x] Viewport에 Grid 선 표시됨
- [x] 하단에 Content 브라우저 표시됨
- [ ] Content → Isaac Sim → Robots 에서 로봇 모델 확인 가능

## Troubleshooting

### Driver 호환성
현재 Driver 570.211.01 설치됨. 5.1.0 권장은 580.65.06+이지만,
RTX PRO 6000 Blackwell은 570 브랜치가 해당 GPU용 드라이버이므로 정상 작동 예상.
문제 발생 시 드라이버 업데이트 필요.

### CUDA 라이브러리 충돌 (torch/cusparse)
첫 실행 시 아래 에러가 다수 출력됩니다 (ML 관련 확장들):
```
libcusparse.so.12: undefined symbol: __nvJitLinkCreate_12_8, version libnvJitLink.so.12
```
영향받는 확장: `isaacsim.sensors.physx`, `isaacsim.ros2.bridge`, OGN 노드 등.
IsaacSim 번들 PyTorch(CUDA 12.6)와 시스템 CUDA 12.8 라이브러리 버전 충돌이 원인입니다.

**해결 방법**: nvjitlink 패키지를 시스템 CUDA 버전에 맞게 업그레이드:
```bash
conda activate isaac_sim
pip install --upgrade nvidia-nvjitlink-cu12
```
> torch가 12.6.85를 요구하지만 상위 호환되므로 동작에 문제 없음.

### ROS2 Bridge 실행 실패
`isaacsim.ros2.bridge` 확장이 `libament_index_cpp.so`를 찾지 못해 실패:
```
Could not load the dynamic library from .../jazzy/lib/librmw_implementation.so.
Error: libament_index_cpp.so: cannot open shared object file: No such file or directory
```
**해결**: IsaacSim 실행 전 ROS2 환경변수 설정 필요 (Step 1.4 "ROS2 연동 실행" 참고)

### glibc TLS 에러로 실행 안됨
`LD_PRELOAD` 또는 `LD_LIBRARY_PATH` 환경변수가 설정되어 있으면 발생.
```bash
unset LD_PRELOAD
unset LD_LIBRARY_PATH
isaacsim
```

### ROS2 자동 로드
Ubuntu 24.04에서는 ROS 2 Jazzy internal libs가 자동으로 로드됩니다.

---
**Status**: COMPLETED
**Next**: [Step 2: Import URDF](02-import-urdf.md)
