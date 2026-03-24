# Step 8: ApexNAV Object Navigation

## Overview

이 가이드에서는 IsaacSim 상의 FFW-SG2 로봇에 **ApexNAV** 자율 물체 탐색 시스템을 연동합니다.

ApexNAV란? **Zero-Shot Object Navigation** 시스템입니다. 로봇이 사전 학습 없이
**"chair를 찾아가"** 같은 텍스트 명령만으로 환경을 탐색하고 목표 물체를 찾아 이동합니다.

Nav2(Step 7)가 **좌표 기반** 자율 주행이라면, ApexNAV는 **물체 기반** 자율 주행입니다.

이 가이드를 끝내면:
- "go to the chair" 같은 명령으로 로봇이 물체를 찾아 이동
- VLM(Vision-Language Model)으로 물체를 인식하는 과정을 이해
- ApexNAV의 Semantic Frontier Exploration이 어떻게 동작하는지 이해

> **참고**: ApexNAV는 HKUST(GZ)의 Robotics-STAR Lab에서 개발한
> [논문](https://ieeexplore.ieee.org/document/10916482) 기반 시스템입니다.
> 이 가이드는 Habitat 시뮬레이터 대신 IsaacSim을 사용하도록 재구성한 것입니다.

### 가이드 구조

| 섹션 | 내용 | 할 일 |
|------|------|-------|
| [1] 개념 이해 | ApexNAV가 뭔지, 어떻게 동작하는지 | 읽기 |
| [2] 사전 준비 | 패키지 설치, VLM 서버 빌드 | **터미널에서 실행** |
| [3] 아키텍처 | 전체 시스템 구조와 데이터 흐름 | 읽기 |
| [4] 브릿지 설정 | IsaacSim ↔ ApexNAV 연결 | **터미널에서 실행** |
| [5] 실행 | 전체 파이프라인 실행 | **IsaacSim + 터미널** |
| [6] 설정 파일 | 파라미터 설명 | 필요할 때 참고 |
| [7] 트러블슈팅 | 안 될 때 해결법 | 필요할 때 참고 |

## Prerequisites

- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 완료 (Step 2)
- [x] 센서 구성 완료 — 특히 **ZED Mini RGB + Depth** (Step 3)
- [x] TF 발행 완료 (Step 4)
- [x] 관절 제어 완료 (Step 5)
- [x] Swerve Drive 동작 확인 (Step 6)
- [x] ROS2 Jazzy 환경 (`conda deactivate` + `source ros2-bridge-env.sh`)
- [x] ApexNAV ROS2 Wrapper 빌드 완료

> **Step 7(Nav2)은 필수가 아닙니다.** ApexNAV는 자체 경로 계획 시스템을 사용합니다.
> Nav2와 ApexNAV를 **동시에 실행하지 마세요** — 둘 다 `/cmd_vel`을 발행하므로 충돌합니다.

---

## [1] ApexNAV 개념 이해

### ApexNAV가 하는 일

```
사용자: "chair를 찾아가!"
    ↓
ApexNAV:
    1. 환경 탐색 (어디에 뭐가 있는지 모르니까)
    2. VLM으로 "이게 chair인가?" 판단
    3. 확신이 들면 그쪽으로 이동
    4. 도착!
```

### Nav2와의 차이

| 항목 | Nav2 (Step 7) | ApexNAV (Step 8) |
|------|--------------|------------------|
| 목표 | **좌표** (x=3.0, y=2.0) | **물체** ("chair") |
| 지도 | SLAM으로 미리 만듦 | **실시간** depth 기반 SDF 맵 |
| 탐색 | 기하학적 frontier | 기하학적 + **의미론적** frontier (semantic value) |
| 물체 인식 | 없음 | VLM (GroundingDINO, BLIP2, YOLOv7) |
| 판단 | 없음 | LLM (물체 혼동 방지 + 방 추론) |

> **용어 설명:**
> - **SDF 맵 (Signed Distance Field)**: 각 셀에 가장 가까운 장애물까지의 거리를 저장하는 지도. depth 카메라 데이터로 실시간 생성됩니다.
> - **Frontier**: 탐사한 영역과 미탐사 영역의 경계선. "아직 안 가본 곳의 입구"라고 생각하면 됩니다.
> - **VLM (Vision-Language Model)**: 이미지와 텍스트를 함께 이해하는 AI 모델. "이 사진에 의자가 있나요?"를 판단할 수 있습니다.

### 핵심 기술 3가지

**1. Target-Centric Semantic Fusion (타겟 중심 의미론적 융합)**

VLM은 가끔 틀립니다 (침대를 소파로 인식). ApexNAV는 **여러 각도에서 관찰한 결과를 2D Value Map에 누적**하여 신뢰도를 높입니다.

핵심: 카메라 **FOV(Field of View, 시야각)** 중심부에서 본 것에 높은 가중치를 부여합니다 (`cos²` 가중치). 가장자리에서 흘낏 본 것은 낮은 가중치.

```
관찰 1: "저게 소파인가?" → ITM 점수 0.4 (FOV 가장자리, cos²=0.3)
관찰 2: "저게 소파인가?" → ITM 점수 0.7 (FOV 중심부, cos²=0.9)
관찰 3: "저게 소파인가?" → ITM 점수 0.3 (FOV 가장자리, cos²=0.2)
→ Value Map에 신뢰도 제곱 가중 융합으로 누적
→ 이 영역의 semantic value = 높음 → frontier 탐색 시 우선 방문
```

> **ITM (Image-Text Matching)**: "이 이미지에 [물체]가 있나?" 질문에 0~1 점수로 답하는 모델 (BLIP2).

**2. Adaptive Exploration Strategy (적응형 탐색 전략)**

Frontier를 선택할 때 4가지 전략을 상황에 따라 자동 전환합니다:

| 전략 | 조건 | 동작 |
|------|------|------|
| **Distance-based** | frontier 간 semantic value 차이가 작을 때 | 가장 가까운 frontier 선택 |
| **Semantic-based** | 특정 frontier의 value가 뚜렷이 높을 때 | 해당 frontier 선택 |
| **Hybrid** | value와 거리를 모두 고려해야 할 때 | 거리 + value 가중 조합 |
| **TSP-Optimized** | 여러 유망 frontier가 있을 때 | LKH solver로 최적 순회 경로 계획 |

이 외에 FSM 수준에서 **초기 360도 회전** (주변 파악)과 **물체 직접 접근** (확인된 물체로 이동) 동작도 있습니다.

**3. VLM + LLM 통합**

물체 탐지와 장면 평가가 **별도 경로**로 동작합니다:

**탐지 경로** (물체를 찾아서 3D 위치 추정):

| 단계 | 모델 | 역할 |
|------|------|------|
| 1 | YOLOv7 **또는** GroundingDINO | 물체 탐지 — COCO 클래스는 YOLOv7 (빠름), 나머지는 GroundingDINO (텍스트 기반) |
| 2 | MobileSAM | 세그멘테이션 — 탐지된 물체의 정확한 윤곽(마스크)을 추출 |
| 3 | depth 투영 | 마스크 + depth → 3D 포인트 클라우드 (물체의 공간 위치) |

> **세그멘테이션(Segmentation)**: 이미지에서 물체의 정확한 윤곽을 픽셀 단위로 잘라내는 것.

**장면 평가 경로** (이 방향에 목표 물체가 있을 가능성 평가):

| 모델 | 역할 | 예시 |
|------|------|------|
| BLIP2-ITM | 이미지-텍스트 매칭 | "이 장면에 kitchen이나 chair가 보이나?" → 0.85 |

이 점수가 Value Map에 누적되어 frontier의 semantic value를 결정합니다.

**LLM 보조** (탐지 전 사전 정보 제공):

LLM은 3가지 정보를 반환합니다:
1. **유사 물체 리스트** — "chair와 혼동될 수 있는 것: stool, bench, armchair" → 탐지 대상 확장
2. **동적 신뢰도 임계값** — 물체 난이도에 따라 0.25~0.65 사이 조정
3. **방 추정** — "chair는 주로 living room, dining room에 있음" → BLIP2 질의에 활용

---

## [2] 사전 준비

### 2.1 ApexNAV ROS2 Wrapper

```bash
# 이미 클론되어 있지 않다면:
cd ~
git clone https://github.com/romaster93/ApexNav_ROS2_wrapper.git
```

### 2.2 C++ 플래너 빌드

```bash
cd ~/ApexNav_ROS2_wrapper
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 빌드 시 `plan_env`, `exploration_manager`, `trajectory_manager` 등 패키지가 빌드됩니다.

### 2.3 Python 의존성

```bash
# ApexNAV conda 환경 활성화
conda activate apexnav_ros2

# 주요 의존성 확인
pip install transformers timm opencv-python open3d ultralytics fastapi uvicorn ollama mobile_sam groundingdino
```

### 2.4 VLM 서버 준비

ApexNAV는 4개의 VLM 서버를 **별도 프로세스**로 실행합니다:

| 서버 | 포트 | 용도 |
|------|------|------|
| YOLOv7 | 12184 | COCO 클래스 물체 탐지 |
| GroundingDINO | 12181 | 텍스트 기반 물체 탐지 |
| BLIP2-ITM | 12182 | 이미지-텍스트 매칭 |
| MobileSAM | 12183 | 세그멘테이션 마스크 |

```bash
# 각각 별도 터미널에서:
cd ~/ApexNav_ROS2_wrapper

# YOLOv7
python vlm/detector/yolov7.py

# GroundingDINO
python vlm/detector/grounding_dino.py

# BLIP2-ITM
python vlm/itm/blip2itm.py

# MobileSAM
python vlm/segmentor/sam.py
```

> **GPU 메모리**: 4개 서버 + IsaacSim 동시 실행 시 약 20GB+ 필요합니다.
> RTX PRO 6000 (48GB) 등 대용량 GPU를 권장합니다.

### 2.5 LLM 설정 (Ollama)

```bash
# Ollama 설치 (이미 설치되어 있다면 건너뛰기)
curl -fsSL https://ollama.com/install.sh | sh

# 모델 다운로드
ollama pull qwen3:8b
```

### 2.6 IsaacSim 카메라 설정

ZED Mini에서 **RGB + Depth** 두 개의 토픽이 발행되어야 합니다:

| 토픽 | 타입 | 해상도 | 설정 |
|------|------|--------|------|
| `/zed_mini/rgb` | Image (rgb8) | 640x480 | Camera Helper type: `rgb` |
| `/zed_mini/depth` | Image (32FC1) | 640x480 | Camera Helper type: `depth` |

> **주의**: RGB와 Depth의 해상도가 **반드시 동일**해야 합니다 (640x480).
> Camera Helper 노드의 Render Product에서 width/height를 맞추세요.

---

## [3] 아키텍처

### 전체 시스템 구조

```
┌─────────────────────────────────────────────────────────────────┐
│                        IsaacSim (Docker)                        │
│  Joint State Publisher → /joint_states                          │
│  TF Publisher → World → world → base_link                      │
│  ZED Mini RGB → /zed_mini/rgb                                  │
│  ZED Mini Depth → /zed_mini/depth                              │
└─────────────────────────────────────────────────────────────────┘
                              │ FastDDS UDP
┌─────────────────────────────┼───────────────────────────────────┐
│                          호스트                                  │
│                              │                                  │
│  swerve_controller.py        │                                  │
│    /cmd_vel → IK → IsaacSim  │                                  │
│    /joint_states → FK → /odom│                                  │
│                              │                                  │
│  nav2_bridge.py              │                                  │
│    Static TF: odom → World   │                                  │
│                              ▼                                  │
│  isaacsim_apexnav_bridge.py ◄─── /odom, /zed_mini/*            │
│    /odom → /habitat/odom                                        │
│    /odom → /habitat/sensor_pose (좌표 변환)                      │
│    /zed_mini/rgb → /habitat/camera_rgb                          │
│    /zed_mini/depth → /habitat/camera_depth (정규화)              │
│         │                                                       │
│         ▼                                                       │
│  ApexNAV C++ Planner (exploration_traj mode)                    │
│    exploration_node ← /habitat/odom, depth, sensor_pose         │
│      SDF Map 빌드 → Frontier 탐색 → 경로 계획                    │
│    traj_server → /cmd_vel                                       │
│         │                                                       │
│  isaacsim_realworld_node.py                                     │
│    /habitat/camera_rgb + depth + sensor_pose → VLM 처리         │
│    → /detector/clouds_with_scores (물체 포인트 클라우드)          │
│    → /blip2/cosine_score (의미론적 점수)                         │
│         │                                                       │
│  target_label_publisher.py                                      │
│    사용자 입력 → /detector/label + trigger                       │
│                                                                 │
│  VLM 서버 (FastAPI, GPU)                                        │
│    YOLOv7(:12184), GroundingDINO(:12181)                        │
│    BLIP2(:12182), MobileSAM(:12183)                             │
└─────────────────────────────────────────────────────────────────┘
```

### 데이터 흐름

```
"go to the chair"
    │
    ▼
target_label_publisher
    ├→ /detector/label ("chair")
    ├→ /detector/confidence_threshold (0.3)  ← FSM 시작 조건
    └→ /move_base_simple/goal (트리거)
    │
    ▼
isaacsim_apexnav_bridge
    ├→ /habitat/odom           ← /odom (frame_id 변경)
    ├→ /habitat/sensor_pose    ← /odom (좌표 변환 + 카메라 높이)
    ├→ /habitat/camera_rgb     ← /zed_mini/rgb
    └→ /habitat/camera_depth   ← /zed_mini/depth (정규화 [0,1])
    │
    ▼                              ▼
exploration_node (C++)        isaacsim_realworld_node (Python)
    │                              │
    ├ depth+pose → SDF 맵          ├ RGB+depth+pose → VLM 서버
    ├ frontier 탐색                ├ 물체 탐지 + 세그멘테이션
    ├ semantic value map           ├ 3D 포인트 클라우드 생성
    ├ 경로 계획                    └→ /detector/clouds_with_scores
    │                                 /blip2/cosine_score
    ▼
traj_server → /cmd_vel → swerve_controller → IsaacSim
```

### 토픽 매핑 테이블

| IsaacSim 토픽 | ApexNAV 토픽 | 변환 내용 |
|---|---|---|
| `/odom` | `/habitat/odom` | frame_id: `odom` → `world` |
| `/odom` | `/habitat/sensor_pose` | 좌표 변환 + 카메라 높이(0.88m) |
| `/zed_mini/rgb` | `/habitat/camera_rgb` | frame_id → `world` |
| `/zed_mini/depth` | `/habitat/camera_depth` | depth(미터) / 5.0 → [0, 1] 정규화 |
| `/cmd_vel` (traj_server) | `/cmd_vel` (swerve_controller) | 직접 연결, 변환 없음 |

---

## [4] 브릿지 설정

### 4.1 왜 브릿지가 필요한가?

ApexNAV는 원래 **Habitat 시뮬레이터**용으로 만들어졌습니다.
IsaacSim의 토픽을 ApexNAV가 기대하는 `/habitat/*` 형식으로 변환해야 합니다.

**두 가지 핵심 변환:**

1. **Depth 정규화** — IsaacSim은 미터 단위(3.5m = 3.5)로 depth를 발행하지만,
   ApexNAV C++ 플래너는 [0, 1] 정규화된 값을 기대합니다.

   ```
   IsaacSim: 2.5m → 브릿지: 2.5 / 5.0 = 0.5 → ApexNAV: 0.5 × 5.0 = 2.5m (복원)
   ```

2. **좌표 변환 (Habitat Forward Transform)** — ApexNAV의 VLM 파이프라인은
   내부적으로 Habitat 좌표계를 사용합니다. 브릿지에서 Habitat의 좌표 변환을
   적용하면, 기존 코드를 수정하지 않고 정확한 3D 물체 위치를 얻을 수 있습니다.

> **좌표 변환 원리 (Option A)**:
> 브릿지가 Habitat publisher의 forward transform을 적용 →
> `real_world_test_habitat.py`의 inverse transform이 정상 역변환 →
> `get_object_point_cloud()`가 올바른 3D 좌표 생성.
> 이 방식으로 ApexNAV 코드를 **한 줄도 수정하지 않습니다**.

### 4.2 isaacsim_apexnav_bridge.py

`scripts/isaacsim_apexnav_bridge.py`가 하는 일:

| 입력 | 출력 | 변환 |
|------|------|------|
| `/odom` | `/habitat/odom` | frame_id만 변경 |
| `/odom` | `/habitat/sensor_pose` | Habitat forward transform 적용 |
| `/zed_mini/rgb` | `/habitat/camera_rgb` | frame_id만 변경 |
| `/zed_mini/depth` | `/habitat/camera_depth` | 미터 → [0,1] 정규화 |

```bash
# 실행
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py
```

정상 시작 시 출력:
```
[INFO] [isaacsim_apexnav_bridge]: IsaacSim↔ApexNAV bridge started (camera_height=0.88m, max_depth=5.0m)
```

> **파라미터:**
> - `camera_height` (기본 0.88m) — Habitat 좌표 변환에 사용되는 값. Habitat 시뮬레이터의 기본 카메라 높이(0.88m)와 일치해야 합니다. 실제 FFW-SG2 카메라 높이(1.58m)와 다르지만, 이 값은 좌표 변환 round-trip을 위한 것이므로 0.88을 유지해야 합니다.
> - `max_depth` (기본 5.0m) — depth 정규화 기준값. C++ 플래너의 `depth_filter_maxdist`(4.99m)과 거의 일치. 이 값으로 나눠 depth를 [0, 1] 범위로 정규화합니다.

### 4.3 동작 확인

```bash
# /habitat 토픽이 발행되는지 확인
ros2 topic list | grep habitat

# 기대 출력:
# /habitat/camera_depth
# /habitat/camera_rgb
# /habitat/odom
# /habitat/sensor_pose

# depth 정규화 확인 (값이 0~1 범위)
ros2 topic echo /habitat/camera_depth --field encoding --once
# → 32FC1

# odom 확인
ros2 topic echo /habitat/odom --field header.frame_id --once
# → world
```

---

## [5] 실행

### 5.1 전체 실행 순서

터미널이 많으므로 **3단계로 나누어** 실행합니다. 각 단계가 동작하는지 확인한 후 다음 단계로 넘어가세요.

> **환경 설정 주의**: 터미널마다 필요한 환경이 다릅니다. 아래 표의 "환경" 열을 꼭 확인하세요.

**Phase A — 기본 인프라 (IsaacSim + 로봇 제어)**

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **1** | `conda activate isaac_sim` | IsaacSim | `isaacsim` → Play |
| **2** | `conda deactivate` + `source ros2-bridge-env.sh` | Swerve Controller | `python3 ~/ms_AIworker/scripts/swerve_controller.py` |
| **3** | `conda deactivate` + `source ros2-bridge-env.sh` | Nav2 Bridge (TF) | `python3 ~/ms_AIworker/scripts/nav2_bridge.py` |
| **4** | `conda deactivate` + `source ros2-bridge-env.sh` | ApexNAV Bridge | `python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py` |

> **확인**: `ros2 topic list | grep habitat` → `/habitat/odom`, `/habitat/camera_rgb` 등이 보여야 합니다.

**Phase B — VLM/LLM 서버 (AI 모델)**

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **5** | `conda activate apexnav_ros2` | YOLOv7 | `cd ~/ApexNav_ROS2_wrapper && python vlm/detector/yolov7.py` |
| **6** | `conda activate apexnav_ros2` | GroundingDINO | `cd ~/ApexNav_ROS2_wrapper && python vlm/detector/grounding_dino.py` |
| **7** | `conda activate apexnav_ros2` | BLIP2-ITM | `cd ~/ApexNav_ROS2_wrapper && python vlm/itm/blip2itm.py` |
| **8** | `conda activate apexnav_ros2` | MobileSAM | `cd ~/ApexNav_ROS2_wrapper && python vlm/segmentor/sam.py` |

> **확인**: 각 서버가 `Uvicorn running on http://0.0.0.0:121xx` 메시지를 출력하면 준비 완료.

**Phase C — ApexNAV 플래너 + 실행**

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **9** | `conda deactivate` + `source ros2-bridge-env.sh` + `source ~/ApexNav_ROS2_wrapper/install/setup.bash` | C++ 플래너 | `ros2 launch exploration_manager exploration_isaacsim.launch.py` |
| **10** | `conda activate apexnav_ros2` | VLM 노드 | `python3 ~/ms_AIworker/scripts/isaacsim_realworld_node.py` |
| **11** | `conda deactivate` + `source ros2-bridge-env.sh` | RViz | `rviz2 --ros-args -p use_sim_time:=true` |
| **12** | `conda deactivate` + `source ros2-bridge-env.sh` | 물체 명령 | `python3 ~/ms_AIworker/scripts/target_label_publisher.py` |

> **확인**: 터미널 9에서 `Exploration FSM initialized` 메시지가 나오면 준비 완료.

### 5.2 C++ 플래너 실행

```bash
cd ~/ApexNav_ROS2_wrapper
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch exploration_manager exploration_isaacsim.launch.py
```

> 정상 시작 시 `[exploration_node]`, `[traj_server]` 노드가 올라옵니다.
> `Managed nodes are active` 또는 `Exploration FSM initialized` 메시지가 나오면 준비 완료.

### 5.3 물체 탐색 시작

```bash
# 터미널 12에서:
python3 ~/ms_AIworker/scripts/target_label_publisher.py

# 대화형 모드:
target> chair
# → /detector/label, /move_base_simple/goal 발행
# → 로봇이 탐색 시작!

# 또는 직접 지정:
python3 ~/ms_AIworker/scripts/target_label_publisher.py chair
```

### 5.4 RViz에서 확인

RViz에서 다음을 추가하면 탐색 과정을 시각화할 수 있습니다:

- `/habitat/camera_rgb` → Image
- `/visualization_marker` → Marker (탐색 경로)
- `/exploration_marker` → Marker (frontier 시각화)
- TF → 로봇 위치

### 5.5 탐색 과정

IsaacSim은 궤적 모드(`is_real_world=true`)를 사용합니다. FSM 상태:

```
1. [INIT]          FSM 초기화, /detector/confidence_threshold 수신 대기
2. [WAIT_TRIGGER]  /move_base_simple/goal 수신 대기
3. [PLAN_TRAJ]     SDF 맵 빌드, frontier 탐색, 연속 궤적 계획
4. [EXEC_TRAJ]     traj_server가 궤적 추종 → /cmd_vel 발행
5. [REPLAN]        실행 중 새 정보 반영하여 궤적 재계획
6. [반복]          3-5를 반복하며 환경 탐색
7. [FINISH]        물체 발견 + 접근 완료 → 정지
```

> **참고**: Habitat 시뮬레이터 모드(`is_real_world=false`)에서는 이산 액션(FORWARD 0.25m, LEFT 30도)을
> 사용하지만, IsaacSim에서는 연속 궤적 기반 제어를 사용합니다.

---

## [6] 설정 파일 설명

### planning_param_ffw.yaml (로봇 물리 파라미터)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `length` | 0.56 | 로봇 전후 길이 (m) |
| `width` | 0.51 | 로봇 좌우 폭 (m) |
| `wheel_base` | 0.43 | 축간 거리 (m) |
| `safe_dist` | 0.45 | 안전 거리 (m) |
| `max_vel` | 1.0 | 최대 속도 (m/s) |
| `max_acc` | 2.0 | 최대 가속도 (m/s^2) |

> 이 값들은 FFW-SG2의 실제 바퀴 위치(`swerve_controller.py`)에서 계산되었습니다.

### exploration_isaacsim.launch.py (플래너 파라미터)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `map_size_x/y` | 30.0 | SDF 맵 크기 (m) |
| `obstacles_inflation` | 0.45 | 장애물 팽창 반경 (m) — 로봇 반경 |
| `depth_filter_maxdist` | 5.0 | depth 최대 거리 (m) — 브릿지 정규화와 일치 |
| `is_real_world` | true | 궤적 모드 사용 (이산 액션 대신 연속 속도) |
| `cx, cy` | 320, 240 | 카메라 주점 (640x480 기준) |
| `fx, fy` | 388.19, 422.05 | 카메라 초점 거리 (hfov=79도, 640x480에서 계산) |

> **fx, fy 값**: IsaacSim에서 `/zed_mini/camera_info` 토픽의 K 행렬을 확인하여
> 정확한 값으로 수정하세요.

### isaacsim_realworld.yaml (VLM/LLM 설정)

| 섹션 | 파라미터 | 값 | 설명 |
|------|---------|-----|------|
| depth_sensor | `max_depth` | 5.0 | 최대 depth 범위 (m) |
| depth_sensor | `height/width` | 480/640 | 해상도 |
| detector.yolo | `confidence_threshold` | 0.3 | YOLO 탐지 신뢰도 |
| detector.groundingDINO | `confidence_threshold` | 0.40 | GroundingDINO 신뢰도 |
| llm | `llm_client` | ollama | LLM 백엔드 (ollama/deepseek) |
| llm | `ollama` | qwen3:8b | Ollama 모델 |

---

## [7] Troubleshooting

### 로봇이 안 움직임 (FSM이 INIT에서 멈춤)

- `/detector/confidence_threshold`가 발행되고 있는지 확인:
  ```bash
  ros2 topic hz /detector/confidence_threshold
  ```
  0Hz면 `target_label_publisher.py`가 실행 중인지 확인

- `/move_base_simple/goal`이 발행되었는지 확인 (FSM 트리거)

### depth 맵이 깨짐 (SDF 맵이 이상함)

- depth 정규화 확인:
  ```bash
  # 값이 0~1 범위인지 확인
  python3 -c "
  import rclpy; from sensor_msgs.msg import Image; import numpy as np
  rclpy.init(); node = rclpy.create_node('check')
  msg = None
  def cb(m): global msg; msg = m
  node.create_subscription(Image, '/habitat/camera_depth', cb, 10)
  rclpy.spin_once(node, timeout_sec=3.0)
  if msg:
      d = np.frombuffer(msg.data, dtype=np.float32)
      print(f'min={d.min():.3f} max={d.max():.3f}')  # 0~1 범위여야 함
  rclpy.shutdown()
  "
  ```
- 값이 1.0 초과면 브릿지의 정규화가 안 되고 있는 것

### VLM 물체 위치가 틀림

- 좌표 변환 round-trip 확인:
  ```bash
  # /habitat/sensor_pose의 z값이 0.88 근처인지
  ros2 topic echo /habitat/sensor_pose --field pose.pose.position.z --once
  # → 0.88 근처여야 함
  ```
- z가 1.58이면 bridge의 `camera_height` 파라미터가 잘못된 것 (0.88이어야 함)

### VLM 서버 연결 실패

- 각 서버가 해당 포트에서 실행 중인지 확인:
  ```bash
  curl http://localhost:12184/health  # YOLOv7
  curl http://localhost:12181/health  # GroundingDINO
  curl http://localhost:12182/health  # BLIP2
  curl http://localhost:12183/health  # MobileSAM
  ```

### /cmd_vel이 0으로만 나옴

- C++ 플래너가 실행 중인지 확인
- `/habitat/odom`, `/habitat/camera_depth` 토픽이 들어오는지 확인
- `traj_server` 로그에 에러 없는지 확인

### RViz에서 Depth 이미지가 깨져 보임

32FC1 depth 이미지가 세로 줄무늬로 보이는 경우:
1. Image display → **Normalize Range** 체크 해제
2. **Min Value**: 0.0, **Max Value**: 1.0 (정규화된 값이므로)

또는 `rqt_image_view`로 확인:
```bash
ros2 run rqt_image_view rqt_image_view /habitat/camera_depth
```

### 실제 ROBOTIS / Habitat과의 차이점

| 항목 | Habitat 시뮬레이터 | IsaacSim |
|------|-------------------|----------|
| 시뮬레이터 | Habitat-Sim | NVIDIA IsaacSim 5.1.0 |
| 로봇 제어 | 이산 액션 (0.25m/30도) | 연속 속도 (`/cmd_vel`) |
| 깊이 센서 | 정규화 [0,1] 직접 출력 | 미터 단위 → 브릿지에서 정규화 |
| 좌표계 | Habitat GPS (XZY) | ROS 표준 (XYZ) → 브릿지에서 변환 |
| 물리 엔진 | Bullet (기본) | PhysX 5 |
| 제어 모드 | `exploration_fsm` (이산) | `exploration_fsm_traj` (궤적) |

---

**Status**: 구현 완료 — 테스트 진행 중
**이전**: [Step 7: Navigation System (Nav2)](07-navigation-system.md)
**다음**: (완료 후 추가 예정)
