# Step 8: ApexNAV 개요

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
| [4] 브릿지 설정 | IsaacSim <-> ApexNAV 연결 | **터미널에서 실행** |
| [5] 실행 | 전체 파이프라인 실행 | **IsaacSim + 터미널** |
| [6] 설정 파일 | 파라미터 설명 | 필요할 때 참고 |
| [7] 트러블슈팅 | 안 될 때 해결법 | 필요할 때 참고 |

## Prerequisites

- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 완료 (Step 3)
- [x] 센서 구성 완료 — 특히 **ZED Mini RGB + Depth** (Step 4)
- [x] TF 발행 완료 (Step 5)
- [x] 관절 제어 완료 (Step 6)
- [x] Swerve Drive 동작 확인 (Step 7)
- [x] ROS2 Jazzy 환경 (`conda deactivate` + `source ros2-bridge-env.sh`)
- [x] ApexNAV ROS2 Wrapper 빌드 완료

> **Step 7(Nav2)은 필수가 아닙니다.** ApexNAV는 자체 경로 계획 시스템을 사용합니다.
> Nav2와 ApexNAV를 **동시에 실행하지 마세요** — 둘 다 `/cmd_vel`을 발행하므로 충돌합니다.

---

## [1] ApexNAV 개념 이해

### ApexNAV가 하는 일

```
사용자: "chair를 찾아가!"
    |
ApexNAV:
    1. 환경 탐색 (어디에 뭐가 있는지 모르니까)
    2. VLM으로 "이게 chair인가?" 판단
    3. 확신이 들면 그쪽으로 이동
    4. 도착!
```

### Nav2와의 차이

| 항목 | Nav2 (Step 7) | ApexNAV (Step 8~11) |
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

핵심: 카메라 **FOV(Field of View, 시야각)** 중심부에서 본 것에 높은 가중치를 부여합니다 (`cos^2` 가중치). 가장자리에서 흘낏 본 것은 낮은 가중치.

```
관찰 1: "저게 소파인가?" -> ITM 점수 0.4 (FOV 가장자리, cos^2=0.3)
관찰 2: "저게 소파인가?" -> ITM 점수 0.7 (FOV 중심부, cos^2=0.9)
관찰 3: "저게 소파인가?" -> ITM 점수 0.3 (FOV 가장자리, cos^2=0.2)
-> Value Map에 신뢰도 제곱 가중 융합으로 누적
-> 이 영역의 semantic value = 높음 -> frontier 탐색 시 우선 방문
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
| 3 | depth 투영 | 마스크 + depth -> 3D 포인트 클라우드 (물체의 공간 위치) |

> **세그멘테이션(Segmentation)**: 이미지에서 물체의 정확한 윤곽을 픽셀 단위로 잘라내는 것.

**장면 평가 경로** (이 방향에 목표 물체가 있을 가능성 평가):

| 모델 | 역할 | 예시 |
|------|------|------|
| BLIP2-ITM | 이미지-텍스트 매칭 | "이 장면에 kitchen이나 chair가 보이나?" -> 0.85 |

이 점수가 Value Map에 누적되어 frontier의 semantic value를 결정합니다.

**LLM 보조** (탐지 전 사전 정보 제공):

LLM은 3가지 정보를 반환합니다:
1. **유사 물체 리스트** — "chair와 혼동될 수 있는 것: stool, bench, armchair" -> 탐지 대상 확장
2. **동적 신뢰도 임계값** — 물체 난이도에 따라 0.25~0.65 사이 조정
3. **방 추정** — "chair는 주로 living room, dining room에 있음" -> BLIP2 질의에 활용

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

**이전**: [Step 7: Navigation System](07-navigation-system.md) | **다음**: [Step 9: ApexNAV 브릿지](09-apexnav-bridge.md)
