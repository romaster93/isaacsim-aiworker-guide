# Step 9: ApexNAV 브릿지 설정

## [3] 아키텍처

### 전체 시스템 구조

```
┌─────────────────────────────────────────────────────────────────┐
│                        IsaacSim (Docker)                        │
│  Joint State Publisher → /joint_states                          │
│  TF Publisher → World → world → base_link → head_link2         │
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
│  isaacsim_apexnav_bridge.py ◄─── /odom, /zed_mini/*, TF        │
│    TF(World→base_link) → /habitat/odom        (실제 위치)       │
│    TF(World→base_link) → /habitat/camera_pose (실제 카메라 pose)│
│    TF(World→base_link) → /habitat/sensor_pose (Habitat 형식)   │
│    /zed_mini/rgb → /habitat/camera_rgb                          │
│    /zed_mini/depth → /habitat/camera_depth (정규화)              │
│         │                                                       │
│         ▼                                                       │
│  ApexNAV C++ Planner (exploration_traj mode)                    │
│    exploration_node ← /habitat/odom, depth, camera_pose         │
│      SDF Map 빌드 (실시간 depth 기반)                             │
│      → KinoAstar (경로 탐색)                                     │
│      → GCopter (궤적 최적화)                                     │
│      → Frontier 탐색 (Hybrid Mode)                               │
│    traj_server → /cmd_vel (MPC 궤적 추종)                        │
│         │                                                       │
│  isaacsim_realworld_node.py (Phase 3)                           │
│    /habitat/camera_rgb + depth + sensor_pose → VLM 처리         │
│    → /detector/clouds_with_scores (물체 포인트 클라우드)          │
│    → /blip2/cosine_score (의미론적 점수)                         │
│         │                                                       │
│  target_label_publisher.py                                      │
│    사용자 입력 → /detector/label + trigger                       │
│                                                                 │
│  VLM 서버 (FastAPI, GPU) — Phase 3에서 사용                      │
│    YOLOv7(:12184), GroundingDINO(:12181)                        │
│    BLIP2(:12182), MobileSAM(:12183)                             │
└─────────────────────────────────────────────────────────────────┘
```

### ApexNAV 플래너 구성 (Nav2 미사용)

ApexNAV는 자체 C++ 플래너를 사용합니다. Nav2는 **전혀 사용하지 않습니다**.

| 구성요소 | 역할 | 상세 |
|---------|------|------|
| **SDF Map** | 실시간 지도 생성 | depth 카메라 데이터로 Signed Distance Field 맵을 실시간 빌드. Nav2의 costmap과 다름 |
| **Frontier 탐색** | 탐색 방향 결정 | Hybrid Mode: 기하학적 frontier + 의미론적 value를 결합하여 탐색 방향 선택 |
| **KinoAstar** | 경로 탐색 | Kinodynamic A* — 로봇 크기, 회전 반경, 속도 제약을 고려한 경로 탐색 |
| **GCopter** | 궤적 최적화 | MINCO 기반 — A* 경로를 부드러운 연속 궤적으로 최적화 |
| **traj_server** | 궤적 추종 | MPC 제어기 — 최적화된 궤적을 추종하여 `/cmd_vel` 발행 |
| **TSP solver** | 순회 최적화 | LKH 기반 — 여러 frontier를 최적 순서로 방문하는 경로 계산 |

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

| 소스 | ApexNAV 토픽 | 변환 내용 | 용도 |
|---|---|---|---|
| TF(World→base_link) | `/habitat/odom` | 실제 위치, frame_id=`World` | C++ FSM 로봇 위치 |
| TF(World→base_link) | `/habitat/camera_pose` | 실제 카메라 pose (height=1.58m, 카메라 방향) | C++ map_ros depth 투영 |
| TF(World→base_link) | `/habitat/sensor_pose` | Habitat forward transform (height=0.88m) | VLM 파이프라인 (Phase 3) |
| `/zed_mini/rgb` | `/habitat/camera_rgb` | frame_id → `World` | VLM 입력 |
| `/zed_mini/depth` | `/habitat/camera_depth` | depth(미터) / 5.0 → [0, 1] 정규화 | C++ SDF 맵 빌드 |
| `/cmd_vel` (traj_server) | `/cmd_vel` (swerve_controller) | 직접 연결, 변환 없음 | 로봇 제어 |

> **카메라 pose vs sensor_pose**: C++ map_ros는 실제 카메라 위치/방향이 필요(`/habitat/camera_pose`).
> VLM 파이프라인은 Habitat 좌표 round-trip이 필요(`/habitat/sensor_pose`). 두 토픽을 별도 발행합니다.

> **좌표 프레임**: 모든 토픽은 `World` 프레임(IsaacSim stage root, 고정)을 사용합니다.
> `world`(소문자)는 articulation root로 로봇과 함께 움직이므로 사용하지 않습니다.

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
| TF(World→base_link) | `/habitat/odom` | 실제 로봇 위치 (World 프레임) |
| TF(World→base_link) | `/habitat/camera_pose` | 실제 카메라 pose (height=1.58m) |
| TF(World→base_link) | `/habitat/sensor_pose` | Habitat forward transform (height=0.88m) |
| `/zed_mini/rgb` | `/habitat/camera_rgb` | frame_id → World |
| `/zed_mini/depth` | `/habitat/camera_depth` | 미터 → [0,1] 정규화 |

```bash
# 공통: ROS2 환경 준비
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

# (1) 기본 실행 (코드 내 기본값 사용)
python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py

# (2) yaml 설정 사용 (권장, Step 10에서 사용하는 방식)
python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py \
    --ros-args --params-file ~/ms_AIworker/config/apexnav_bridge.yaml
```

> ⚠️ **2026-04-09 이후: yaml 통합 설정 권장**
> `config/apexnav_bridge.yaml` 을 `--ros-args --params-file` 로 로드하면, 같은 yaml 을 `algorithm_traj.launch.py` 가 직접 파싱해 C++ 플래너의 `depth_filter_maxdist` 등에 자동 전파합니다. bridge(Python) 와 launch(C++) 가 **single source of truth** 를 공유하므로 수동 일치가 불필요합니다. 수동 일치 방식은 구버전 호환을 위해 유지되지만, 새 설정은 yaml 경유가 안전합니다.

> **위치 소스**: odom FK 적분값이 아닌 **TF lookup(World→base_link)**으로 실제 위치를 가져옵니다.
> odom (0,0) 시작점과 IsaacSim 실제 위치가 다르기 때문입니다.

> **카메라 방향**: head_link2에 부착된 ZED Mini의 실제 방향을 사용합니다.
> 카메라 optical frame → base_link 회전: `q=(-0.5, 0.5, -0.5, 0.5)` (틸트 없음, 정면 직시)
> 카메라 offset (base_link 기준): x=0.066m, z=1.58m

> **파라미터** (`config/apexnav_bridge.yaml` — 실제 2필드 구조):
> ```yaml
> isaacsim_apexnav_bridge:
>   ros__parameters:
>     max_depth: 5.0              # depth 정규화 기준 (meters); launch 가 자동으로 C++ depth_filter_maxdist 에 전파
>     camera_height_habitat: 0.88 # VLM 파이프라인(Phase 3) Habitat round-trip용 (실제 1.58m 아님)
> ```
> - `max_depth` — bridge 는 `depth / max_depth` 로 정규화해서 [0,1] 범위로 보냅니다. yaml 을 사용하면 `algorithm_traj.launch.py` 가 같은 값을 읽어 C++ `depth_filter_maxdist = max_depth - 0.01` 로 전파하므로 수동 일치 불필요합니다.
> - `camera_height_habitat` — VLM 파이프라인의 Habitat 좌표계 round-trip 용 가상 카메라 높이. 실제 ZED Mini 높이(1.58m) 와 다릅니다. 0.88 유지 필요.
>
> 상세 설명은 [Step 10: 설정 파일 설명 — config/apexnav_bridge.yaml](10-apexnav-autonomous.md#6-설정-파일-설명) 을 참고하세요.

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

**이전**: [Step 8: ApexNAV 개요](08-apexnav-overview.md) | **다음**: [Step 10: ApexNAV 자율 주행](10-apexnav-autonomous.md)
