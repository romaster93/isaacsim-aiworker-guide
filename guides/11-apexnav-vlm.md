# Step 11: ApexNAV VLM 통합

## Overview

이 가이드에서는 ApexNAV의 **VLM(Vision-Language Model) 파이프라인**을 IsaacSim과 통합합니다.
Step 10에서 C++ 플래너만으로 기하학적 frontier 탐색이 동작했다면,
이 단계에서는 **의미론적 물체 인식**을 추가하여 목표 물체를 찾아가는 기능을 완성합니다.

> **선행 조건**: Step 10의 Phase A (IsaacSim + 브릿지)와 Phase C (C++ 플래너)가 정상 동작해야 합니다.

---

## [1] VLM 파이프라인 개요

Phase B + Phase C를 추가하면 전체 물체 인식 루프가 완성됩니다.

```
사용자 입력: "chair를 찾아가"
    │
    ├── target_label_publisher.py
    │     → /detector/label ("chair")
    │     → /move_base_simple/goal (FSM 트리거)
    │
    ├── isaacsim_realworld_node.py  ← Phase C (이 가이드)
    │     RGB + depth + sensor_pose 수신
    │     → VLM 서버 4개에 HTTP 요청
    │     → /detector/clouds_with_scores (물체 3D 포인트 클라우드)
    │     → /blip2/cosine_score (의미론적 점수)
    │
    ├── VLM 서버 4개 (FastAPI)  ← Phase B (이 가이드)
    │     YOLOv7        (포트 12184) — COCO 클래스 물체 탐지
    │     GroundingDINO (포트 12181) — 텍스트 기반 물체 탐지
    │     BLIP2-ITM     (포트 12182) — 이미지-텍스트 매칭 점수
    │     MobileSAM     (포트 12183) — 세그멘테이션 마스크
    │
    └── exploration_node (C++)
          /detector/clouds_with_scores → semantic value map 업데이트
          /blip2/cosine_score → frontier 우선순위 결정
          → 물체가 확인되면 직접 접근 → 완료
```

---

## [2] 사전 준비 확인

VLM 서버 실행 전에 확인해야 할 사항들입니다.

### 2.1 conda 환경

```bash
# apexnav_ros2 환경 활성화
conda activate apexnav_ros2

# 주요 패키지 설치 확인
python -c "import torch; print('torch:', torch.__version__, '| CUDA:', torch.cuda.is_available())"
python -c "import transformers; print('transformers:', transformers.__version__)"
python -c "import fastapi, uvicorn; print('fastapi/uvicorn OK')"
```

> **주의**: VLM 서버는 반드시 `conda activate apexnav_ros2` 환경에서 실행해야 합니다.
> ROS2 환경(`source ros2-bridge-env.sh`)은 VLM 서버 터미널에서는 **사용하지 않습니다**.

### 2.2 GPU 메모리 확인

4개 VLM 서버와 IsaacSim을 동시에 실행하면 약 **20GB+ VRAM**이 필요합니다.

```bash
# GPU 메모리 상태 확인
nvidia-smi
```

| 서버 | 모델 | 예상 VRAM |
|------|------|-----------|
| YOLOv7 | YOLOv7-tiny or large | ~1-4GB |
| GroundingDINO | GroundingDINO-Base | ~2GB |
| BLIP2-ITM | BLIP2 | ~8GB |
| MobileSAM | MobileSAM | ~0.5GB |
| IsaacSim | RTX 렌더링 | ~6-8GB |

### 2.3 Ollama 설정

LLM은 VLM 노드 시작 시 물체 탐지 대상 확장 및 방 추론에 사용됩니다.

```bash
# Ollama가 실행 중인지 확인 (base 환경 또는 시스템)
ollama list

# qwen3:8b 모델이 없다면:
ollama pull qwen3:8b

# Ollama 서비스 실행 (백그라운드)
ollama serve &
```

---

## [3] Phase B — VLM 서버 실행

VLM 서버 4개를 **별도 터미널**에서 실행합니다.
모두 `conda activate apexnav_ros2` 환경이 필요합니다.

### 실행 명령

| 터미널 | 환경 | 서버 | 명령어 |
|--------|------|------|--------|
| **5** | `conda activate apexnav_ros2` | YOLOv7 (포트 12184) | `cd ~/ApexNav_ROS2_wrapper && python vlm/detector/yolov7.py` |
| **6** | `conda activate apexnav_ros2` | GroundingDINO (포트 12181) | `cd ~/ApexNav_ROS2_wrapper && python vlm/detector/grounding_dino.py` |
| **7** | `conda activate apexnav_ros2` | BLIP2-ITM (포트 12182) | `cd ~/ApexNav_ROS2_wrapper && python vlm/itm/blip2itm.py` |
| **8** | `conda activate apexnav_ros2` | MobileSAM (포트 12183) | `cd ~/ApexNav_ROS2_wrapper && python vlm/segmentor/sam.py` |

### 각 서버 역할

| 서버 | 역할 | 사용 조건 |
|------|------|-----------|
| **YOLOv7** | COCO 80개 클래스 물체 탐지 (빠름) | "chair", "cup", "bottle" 등 COCO 클래스 |
| **GroundingDINO** | 텍스트 입력 기반 물체 탐지 | COCO에 없는 임의 물체 ("red vase" 등) |
| **BLIP2-ITM** | 이미지-텍스트 매칭 점수 (0~1) | semantic value map 업데이트에 항상 사용 |
| **MobileSAM** | 탐지된 물체의 픽셀 마스크 추출 | 3D 포인트 클라우드 생성에 사용 |

> **YOLOv7 vs GroundingDINO**: COCO 클래스 물체는 YOLOv7이 빠릅니다.
> COCO에 없는 물체(예: "vase", "piano")는 GroundingDINO가 처리합니다.
> `get_object()` 함수가 레이블에 따라 자동으로 선택합니다.

### 시작 확인

각 서버가 준비되면 다음과 같은 메시지가 출력됩니다:

```
INFO:     Uvicorn running on http://0.0.0.0:12184 (Press CTRL+C to quit)
INFO:     Application startup complete.
```

---

## [4] Phase C — VLM 노드 실행

Phase A (Step 10의 기본 인프라)와 Phase B (VLM 서버 4개)가 모두 실행된 상태에서 추가합니다.

### 4.1 실행 명령

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **10** | `conda activate apexnav_ros2` | VLM 노드 | `python3 ~/ms_AIworker/scripts/isaacsim_realworld_node.py` |
| **12** | `conda deactivate` + `source ros2-bridge-env.sh` | 물체 명령 (이미 실행 중) | `python3 ~/ms_AIworker/scripts/target_label_publisher.py` |

> **터미널 12 참고**: `target_label_publisher.py`는 Step 10에서도 사용합니다.
> VLM 없이 탐색할 때도 이 스크립트가 필요합니다.

### 4.2 VLM 노드 동작 원리

`isaacsim_realworld_node.py`는 `real_world_test_habitat.py` 패턴을 기반으로 합니다.
IsaacSim에 맞게 좌표 변환과 QoS 설정을 조정했습니다.

**수신 토픽 (브릿지에서)**:

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/habitat/camera_rgb` | `sensor_msgs/Image` | VLM 물체 탐지 입력 |
| `/habitat/camera_depth` | `sensor_msgs/Image` | 3D 포인트 클라우드 생성용 depth |
| `/habitat/sensor_pose` | `nav_msgs/Odometry` | Habitat 좌표 역변환 (gps, compass 복원) |
| `/habitat/odom` | `nav_msgs/Odometry` | 로봇 위치 (현재는 로깅용) |
| `/detector/label` | `std_msgs/String` | 목표 물체 이름 |

**발행 토픽 (C++ 플래너로)**:

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/detector/clouds_with_scores` | `plan_env/MultipleMasksWithConfidence` | 물체 3D 포인트 클라우드 + 신뢰도 |
| `/blip2/cosine_score` | `std_msgs/Float64` | 이미지-텍스트 매칭 점수 (0~1) |
| `/detector/confidence_threshold` | `std_msgs/Float64` | 1Hz 주기 발행 (FSM INIT 탈출 유지) |
| `/detector/detect_img` | `sensor_msgs/Image` | 탐지 결과 시각화 이미지 |

**처리 파이프라인**:

```
RGB + depth + sensor_pose (동기화, slop=0.05초)
    │
    ├── [탐지 경로]
    │     get_object(label, rgb, detector_cfg, llm_answer)
    │         ├── COCO 클래스 → YOLOv7 → 바운딩 박스
    │         ├── 나머지 → GroundingDINO → 바운딩 박스
    │         └── MobileSAM → 픽셀 마스크
    │     inverse_habitat_publisher_transform(sensor_pose)
    │         → gps = [-pos.y, pos.z - 0.88, -pos.x]
    │         → compass = euler[2] + π/2
    │     get_object_point_cloud(cfg, observations, masks)
    │         → depth + gps/compass → 3D 포인트 클라우드
    │     → /detector/clouds_with_scores
    │
    └── [점수 경로]
          get_itm_message_cosine(rgb, label, room)
              → BLIP2 서버에 HTTP 요청
              → "이미지에 [label]이 있는가?" → 0~1 점수
          → /blip2/cosine_score
```

### 4.3 좌표 변환 (inverse_habitat_publisher_transform)

브릿지(`isaacsim_apexnav_bridge.py`)가 Habitat forward transform을 적용해서 `/habitat/sensor_pose`를 발행합니다.
VLM 노드는 그 역변환으로 Habitat 좌표계의 `gps`와 `compass`를 복원합니다.

```
브릿지 forward transform:
  pos.x = x_ros,  pos.y = y_ros,  pos.z = camera_height (0.88m)
  orient = quaternion_from_euler(π/2, π, yaw_ros + π/2)

VLM 노드 inverse transform:
  gps    = [-pos.y, pos.z - 0.88, -pos.x]
  compass = euler[2] + π/2
```

> **camera_height = 0.88m**: 실제 카메라 높이(1.58m)가 아닌 Habitat round-trip 호환값.
> 브릿지의 `camera_height_habitat` 파라미터와 VLM 노드의 `- 0.88` 역변환이 쌍을 이룹니다.
> 이 값을 바꾸면 3D 물체 위치 추정이 틀려집니다.

---

## [5] 전체 실행 순서 (VLM 포함)

Step 10의 Phase A + C에 Phase B와 VLM 노드를 추가한 전체 순서입니다.

### Phase A — 기본 인프라 (Step 10과 동일)

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **1** | `conda activate isaac_sim` | IsaacSim | `isaacsim` → Play |
| **2** | `conda deactivate` + `source ros2-bridge-env.sh` | Swerve Controller | `python3 ~/ms_AIworker/scripts/swerve_controller.py` |
| **3** | `conda deactivate` + `source ros2-bridge-env.sh` | Nav2 Bridge (TF) | `python3 ~/ms_AIworker/scripts/nav2_bridge.py` |
| **4** | `conda deactivate` + `source ros2-bridge-env.sh` | ApexNAV Bridge | `python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py` |

### Phase B — VLM 서버 (이 가이드 신규)

| 터미널 | 환경 | 서버 | 명령어 |
|--------|------|------|--------|
| **5** | `conda activate apexnav_ros2` | YOLOv7 | `cd ~/ApexNav_ROS2_wrapper && python vlm/detector/yolov7.py` |
| **6** | `conda activate apexnav_ros2` | GroundingDINO | `cd ~/ApexNav_ROS2_wrapper && python vlm/detector/grounding_dino.py` |
| **7** | `conda activate apexnav_ros2` | BLIP2-ITM | `cd ~/ApexNav_ROS2_wrapper && python vlm/itm/blip2itm.py` |
| **8** | `conda activate apexnav_ros2` | MobileSAM | `cd ~/ApexNav_ROS2_wrapper && python vlm/segmentor/sam.py` |

### Phase C — ApexNAV 플래너 + VLM 노드

> **실행 순서**: 터미널 12(물체 명령) → 터미널 10(VLM 노드) → 터미널 9(C++ 플래너) 순서를 권장합니다.

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **11** | `conda deactivate` + `source ros2-bridge-env.sh` | RViz | `rviz2 --ros-args -p use_sim_time:=true` |
| **12** | `conda deactivate` + `source ros2-bridge-env.sh` | 물체 명령 (먼저!) | `python3 ~/ms_AIworker/scripts/target_label_publisher.py` |
| **10** | `conda activate apexnav_ros2` | VLM 노드 | `python3 ~/ms_AIworker/scripts/isaacsim_realworld_node.py` |
| **9** | `conda deactivate` + `source ros2-bridge-env.sh` + `source ~/ApexNav_ROS2_wrapper/install/setup.bash` | C++ 플래너 | `ros2 launch exploration_manager exploration_traj.launch.py` |

> **터미널 10 (VLM 노드)**: Hydra config를 `real_world_test_example/config/isaacsim_realworld.yaml`에서 읽습니다.
> 실행 디렉토리는 어디서든 상관없습니다 (절대 경로로 config를 찾습니다).

### 물체 탐색 시작

```bash
# 터미널 12에서 대화형 모드:
target> chair

# 또는 직접 지정:
python3 ~/ms_AIworker/scripts/target_label_publisher.py "sofa"
```

성공 시 터미널 10 (VLM 노드)에서 다음과 같은 로그가 출력됩니다:

```
[IsaacSimRealWorldNode] Received target label: chair
[IsaacSimRealWorldNode] detect: label=chair
[IsaacSimRealWorldNode] value: cosine=0.423
```

---

## [6] 설정 파일 (isaacsim_realworld.yaml)

`/home/cho/ApexNav_ROS2_wrapper/real_world_test_example/config/isaacsim_realworld.yaml`

### detector 섹션

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `confidence_threshold_yolo` | 0.3 | YOLOv7 탐지 수락 최소 신뢰도. 낮추면 탐지가 늘지만 오탐도 증가 |
| `iou_threshold_yolo` | 0.5 | YOLOv7 NMS IOU 임계값. 겹치는 박스 제거 기준 |
| `agnostic_nms` | true | 클래스 무관 NMS 적용 여부 |
| `confidence_threshold_dino` | 0.40 | GroundingDINO 탐지 수락 최소 신뢰도 |
| `text_threshold` | 0.25 | GroundingDINO 텍스트 토큰 매칭 임계값 |

### llm 섹션

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `llm_client` | `ollama` | LLM 백엔드 선택. `ollama` (로컬), `deepseek` (API), `none` (LLM 미사용) |
| `ollama` | `qwen3:8b` | Ollama 사용 시 모델명. `ollama list`로 설치된 모델 확인 |
| `llm_answer_path` | `llm/answers/llm_answer_mp3d.txt` | LLM 응답 캐시 파일 경로 |
| `llm_response_path` | `llm/answers/llm_response_list.txt` | LLM 파싱 결과 캐시 파일 경로 |

> **llm_client: none**: LLM 없이 동작합니다. 물체 혼동 방지 기능과 방 추론이 비활성화됩니다.
> 빠른 테스트에 유용합니다.

### habitat_sensor 섹션

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `camera_height` | 0.88 | Habitat 좌표 round-trip용 가상 카메라 높이 (m). **변경 금지** |
| `camera_min_depth` | 0.0 | depth 최소값 (m). 0 이하는 무시 |
| `camera_max_depth` | 5.0 | depth 최대값 (m). 브릿지의 `max_depth`와 일치시킬 것 |
| `hfov` | 79 | 수평 시야각 (도). ZED Mini 실측값 |
| `height` / `width` | 480 / 640 | 이미지 해상도. 카메라 설정과 일치시킬 것 |

---

## [7] RViz에서 VLM 결과 확인

Step 10의 기본 토픽 외에 VLM 관련 토픽을 추가합니다.

**물체 탐지 결과**:

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/detector/detect_img` | Image | 탐지 바운딩 박스가 그려진 RGB 이미지 |
| `/blip2/cosine_score` | Float64 | 현재 프레임의 ITM 점수 (rqt_plot으로 확인) |

```bash
# ITM 점수 실시간 모니터링
ros2 topic echo /blip2/cosine_score

# 탐지 이미지 확인
ros2 run rqt_image_view rqt_image_view /detector/detect_img
```

---

## [8] Troubleshooting

### VLM 서버 연결 실패

각 서버가 해당 포트에서 실행 중인지 확인합니다:

```bash
curl http://localhost:12184/  # YOLOv7
curl http://localhost:12181/  # GroundingDINO
curl http://localhost:12182/  # BLIP2-ITM
curl http://localhost:12183/  # MobileSAM
```

응답이 없으면:
1. 해당 터미널의 conda 환경이 `apexnav_ros2`인지 확인
2. 서버 터미널의 에러 메시지 확인 (모델 가중치 경로, 의존성 등)
3. `nvidia-smi`로 GPU 메모리 부족 여부 확인

### VLM 노드에서 "waiting for target label" 반복

`/detector/label` 토픽이 발행되지 않은 것입니다.

```bash
ros2 topic hz /detector/label
```

0Hz면 `target_label_publisher.py`에서 아직 물체 이름을 입력하지 않은 것입니다.
`target>` 프롬프트에서 물체 이름을 입력하세요.

### VLM 노드에서 동기화 실패 (메시지가 들어오지 않음)

```bash
# /habitat 토픽이 발행 중인지 확인
ros2 topic hz /habitat/camera_rgb
ros2 topic hz /habitat/camera_depth
ros2 topic hz /habitat/sensor_pose
```

0Hz면 `isaacsim_apexnav_bridge.py`가 실행 중인지, IsaacSim이 Play 상태인지 확인하세요.

타임스탬프 동기화 문제라면 (3개 토픽의 타임스탬프가 크게 다를 경우):

```bash
# 각 토픽의 타임스탬프 확인
ros2 topic echo /habitat/camera_rgb --field header.stamp --once
ros2 topic echo /habitat/sensor_pose --field header.stamp --once
```

slop 기본값은 0.05초입니다. 차이가 크면 `isaacsim_realworld_node.py`의 `slop=0.05`를 올려볼 수 있습니다.

### 물체 위치가 틀림 (3D 포인트 클라우드가 이상한 곳에 생성)

좌표 변환 round-trip을 확인합니다:

```bash
# /habitat/sensor_pose의 z값이 0.88 근처인지 확인
ros2 topic echo /habitat/sensor_pose --field pose.pose.position.z --once
# → 0.88 근처여야 함
```

z가 1.58m면 브릿지의 `camera_height_habitat` 파라미터가 잘못된 것입니다 (0.88이어야 함).
z가 0에 가까우면 브릿지가 TF를 못 읽고 있는 것 — `nav2_bridge.py`가 실행 중인지 확인하세요.

### CUDA out of memory

4개 서버를 하나씩 실행하면서 `nvidia-smi`로 메모리를 확인합니다.
BLIP2가 가장 많은 메모리를 사용합니다. 부족하면:

1. YOLOv7과 GroundingDINO 중 하나만 실행 (목표 물체가 COCO 클래스면 YOLOv7만으로 충분)
2. BLIP2 대신 `llm_client: none` 설정으로 ITM 비활성화 (semantic value map 없이 기하학적 탐색만)

### LLM (Ollama) 응답이 없음

```bash
# Ollama 서비스 상태 확인
ollama list
curl http://localhost:11434/api/tags

# 서비스가 없으면 실행
ollama serve
```

응답이 느리면 `isaacsim_realworld.yaml`에서 `llm_client: none`으로 설정하면 LLM 없이도 동작합니다.

---

## [9] Habitat vs IsaacSim 차이점

| 항목 | Habitat 시뮬레이터 | IsaacSim |
|------|-------------------|----------|
| 시뮬레이터 | Habitat-Sim | NVIDIA IsaacSim 5.1.0 |
| 로봇 제어 | 이산 액션 (FORWARD 0.25m / LEFT 30도) | 연속 속도 (`/cmd_vel`) |
| depth 출력 | 정규화 [0,1] 직접 출력 | 미터 단위 → 브릿지에서 `/5.0` 정규화 |
| 좌표계 | Habitat GPS (XZY 순서) | ROS 표준 (XYZ) → 브릿지에서 변환 |
| sensor_pose 소스 | Habitat simulator 내부 | TF lookup (World→base_link) |
| 물리 엔진 | Bullet (기본) | PhysX 5 |
| 탐색 모드 | `exploration_fsm` (이산) | `exploration_traj` (연속 궤적) |
| VLM 노드 | `real_world_test_habitat.py` | `isaacsim_realworld_node.py` |

---

## [10] InteriorAgent 테스트 환경

VLM object-goal navigation 테스트를 위해 [InteriorAgent](https://huggingface.co/datasets/spatialverse/InteriorAgent) 데이터셋을 활용합니다.
25개 인테리어 씬, 6,149개 메시, 79개 오브젝트 카테고리, 339개 테스트 시나리오가 준비되어 있습니다.

상세 내용 (씬 인벤토리, USD 호환성 분석, 난이도별 테스트 시나리오, 로봇 배치 스크립트):
**[Step 12: InteriorAgent 데이터셋](12-interioragent-dataset.md)**

---

**Status**: Phase 3 미테스트 — 코드 구조 완성, 실제 실행 확인 필요
- Phase 1: 센서 + 브릿지 연결 (depth, rgb, odom → /habitat/* 토픽) — 완료
- Phase 2: ApexNAV C++ 플래너 (SDF 맵, frontier 탐색, 자율 이동) — 완료
- Phase 3: VLM 통합 (물체 인식 + 의미론적 탐색) — 미테스트

---

**이전**: [Step 10: ApexNAV 자율 주행](10-apexnav-autonomous.md)
**다음**: [Step 12: InteriorAgent 데이터셋](12-interioragent-dataset.md)
