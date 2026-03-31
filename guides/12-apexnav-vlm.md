# Step 12: ApexNAV VLM 통합

## Overview

이 가이드에서는 ApexNAV의 **VLM(Vision-Language Model) 파이프라인**을 IsaacSim과 통합합니다.
Step 11에서 C++ 플래너만으로 기하학적 frontier 탐색이 동작했다면,
이 단계에서는 **의미론적 물체 인식**을 추가하여 목표 물체를 찾아가는 기능을 완성합니다.

---

## Phase B -- VLM/LLM 서버 실행

VLM 서버 4개를 **별도 터미널**에서 실행합니다. 모두 `conda activate apexnav_ros2` 환경이 필요합니다.

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **5** | `conda activate apexnav_ros2` | YOLOv7 | `cd ~/ApexNav_ROS2_wrapper && python vlm/detector/yolov7.py` |
| **6** | `conda activate apexnav_ros2` | GroundingDINO | `cd ~/ApexNav_ROS2_wrapper && python vlm/detector/grounding_dino.py` |
| **7** | `conda activate apexnav_ros2` | BLIP2-ITM | `cd ~/ApexNav_ROS2_wrapper && python vlm/itm/blip2itm.py` |
| **8** | `conda activate apexnav_ros2` | MobileSAM | `cd ~/ApexNav_ROS2_wrapper && python vlm/segmentor/sam.py` |

> **확인**: 각 서버가 `Uvicorn running on http://0.0.0.0:121xx` 메시지를 출력하면 준비 완료.

---

## Phase C -- VLM 노드 실행

Phase A (Step 11의 기본 인프라)와 Phase B (위의 VLM 서버)가 모두 실행된 상태에서 추가합니다.

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **10** | `conda activate apexnav_ros2` | VLM 노드 | `python3 ~/ms_AIworker/scripts/isaacsim_realworld_node.py` |
| **12** | `conda deactivate` + `source ros2-bridge-env.sh` | 물체 명령 | `python3 ~/ms_AIworker/scripts/target_label_publisher.py` |

`isaacsim_realworld_node.py`는 `real_world_test_habitat.py` 패턴을 기반으로 합니다:
- `/habitat/camera_rgb` + `/habitat/camera_depth` + `/habitat/sensor_pose`를 수신
- VLM 서버에 물체 탐지/세그멘테이션/ITM 요청
- 결과를 `/detector/clouds_with_scores`, `/blip2/cosine_score`로 발행
- C++ 플래너가 이 정보를 받아 semantic value map을 업데이트

---

## isaacsim_realworld.yaml 설정

| 섹션 | 파라미터 | 값 | 설명 |
|------|---------|-----|------|
| depth_sensor | `max_depth` | 5.0 | 최대 depth 범위 (m) |
| depth_sensor | `height/width` | 480/640 | 해상도 |
| detector.yolo | `confidence_threshold` | 0.3 | YOLO 탐지 신뢰도 |
| detector.groundingDINO | `confidence_threshold` | 0.40 | GroundingDINO 신뢰도 |
| llm | `llm_client` | ollama | LLM 백엔드 (ollama/deepseek) |
| llm | `ollama` | qwen3:8b | Ollama 모델 |

---

## Troubleshooting

### VLM 물체 위치가 틀림

- 좌표 변환 round-trip 확인:
  ```bash
  # /habitat/sensor_pose의 z값이 0.88 근처인지
  ros2 topic echo /habitat/sensor_pose --field pose.pose.position.z --once
  # → 0.88 근처여야 함
  ```
- z가 1.58이면 bridge의 `camera_height` 파라미터가 잘못된 것 (0.88이어야 함)

> **좌표 변환 원리**: 브릿지가 `camera_height_habitat=0.88m`으로 Habitat forward transform을 적용하면,
> `isaacsim_realworld_node.py`의 inverse transform이 정확히 역변환합니다.
> 실제 카메라 높이(1.58m)가 아닌 0.88m를 사용하는 이유는 round-trip 호환성 때문입니다.

### VLM 서버 연결 실패

- 각 서버가 해당 포트에서 실행 중인지 확인:
  ```bash
  curl http://localhost:12184/health  # YOLOv7
  curl http://localhost:12181/health  # GroundingDINO
  curl http://localhost:12182/health  # BLIP2
  curl http://localhost:12183/health  # MobileSAM
  ```

---

## Habitat vs IsaacSim 차이점

| 항목 | Habitat 시뮬레이터 | IsaacSim |
|------|-------------------|----------|
| 시뮬레이터 | Habitat-Sim | NVIDIA IsaacSim 5.1.0 |
| 로봇 제어 | 이산 액션 (0.25m/30도) | 연속 속도 (`/cmd_vel`) |
| 깊이 센서 | 정규화 [0,1] 직접 출력 | 미터 단위 -> 브릿지에서 정규화 |
| 좌표계 | Habitat GPS (XZY) | ROS 표준 (XYZ) -> 브릿지에서 변환 |
| 물리 엔진 | Bullet (기본) | PhysX 5 |
| 제어 모드 | `exploration_fsm` (이산) | `exploration_fsm_traj` (궤적) |

---

**Status**: Phase 3 미완료 -- 테스트 예정
- Phase 1: 센서 + 브릿지 연결 (depth, rgb, odom -> /habitat/* 토픽)
- Phase 2: ApexNAV 플래너 연결 (SDF 맵, frontier 탐색, 자율 이동)
- Phase 3: VLM 통합 (물체 인식 + 의미론적 탐색) -- 테스트 예정

---

**이전**: [Step 11: ApexNAV 자율 주행](11-apexnav-autonomous.md)
