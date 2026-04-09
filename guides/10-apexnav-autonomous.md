# Step 10: ApexNAV 자율 주행

## [5] 실행

### 5.1 전체 실행 순서

터미널이 많으므로 **단계별로 나누어** 실행합니다. 각 단계가 동작하는지 확인한 후 다음 단계로 넘어가세요.

> **환경 설정 주의**: 터미널마다 필요한 환경이 다릅니다. 아래 표의 "환경" 열을 꼭 확인하세요.

**Phase A -- 기본 인프라 (IsaacSim + 로봇 제어)**

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **1** | `conda activate isaac_sim` | IsaacSim | `isaacsim` -> Play |
| **2** | `conda deactivate` + `source ros2-bridge-env.sh` | Swerve Controller | `python3 ~/ms_AIworker/scripts/swerve_controller.py` |
| **3** | `conda deactivate` + `source ros2-bridge-env.sh` | Nav2 Bridge (TF) | `python3 ~/ms_AIworker/scripts/nav2_bridge.py` |
| **4** | `conda deactivate` + `source ros2-bridge-env.sh` | ApexNAV Bridge | `python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py --ros-args --params-file ~/ms_AIworker/config/apexnav_bridge.yaml` |

> **확인**: `ros2 topic list | grep habitat` -> `/habitat/odom`, `/habitat/camera_rgb` 등이 보여야 합니다.

**Phase B -- VLM 서버 (터미널 5-8, 10 예약)**

> Phase B는 [Step 11: ApexNAV VLM 통합](11-apexnav-vlm.md)에서 실행합니다. 여기서는 VLM 없이 기하학적 frontier 탐색만 수행합니다.

**Phase C -- ApexNAV 플래너 실행**

> **순서 주의**: `target_label_publisher.py`를 **먼저** 실행한 후 C++ 플래너를 실행하세요.
> 플래너가 먼저 뜨면 `/detector/confidence_threshold`가 없어서 `[Real] No odom || No target confidence threshold` 경고가 반복됩니다.

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **11** | `conda deactivate` + `source ros2-bridge-env.sh` | RViz | `rviz2 --ros-args -p use_sim_time:=true` |
| **12** | `conda deactivate` + `source ros2-bridge-env.sh` | 물체 명령 (먼저!) | `python3 ~/ms_AIworker/scripts/target_label_publisher.py` |
| **9** | `conda deactivate` + `source ros2-bridge-env.sh` + `source ~/ApexNav_ROS2_wrapper/install/setup.bash` | C++ 플래너 | `ros2 launch exploration_manager exploration_traj.launch.py` |
| **13** | `conda deactivate` + `source ros2-bridge-env.sh` + `source ~/ApexNav_ROS2_wrapper/install/setup.bash` | Swerve Path Follower (traj_server 대체) | `python3 ~/ms_AIworker/scripts/swerve_path_follower.py` |

> **터미널 13 (swerve_path_follower)**: ApexNAV의 기본 `traj_server`는 unicycle MPC 라서 FFW-SG2 swerve의 vy를 활용 못 합니다. `exploration_traj.launch.py`에서 `traj_server`는 비활성화돼 있으며, 대신 이 노드가 `/planning/trajectory`(PolyTraj septic)를 직접 평가해 lookahead 0.25s pure-pursuit로 holonomic `/cmd_vel` 을 발행합니다. 진행 방향으로 로봇을 회전시켜(`angular.z`) depth 카메라가 항상 전방을 보도록 합니다. `trajectory_manager` msg가 필요하므로 `ApexNAV_ROS2_wrapper/install/setup.bash`를 source 하세요.

> **확인**: 터미널 9에서 `Exploration FSM initialized` 메시지가 나오면 준비 완료.

> **VLM 통합** (Phase B + VLM 노드)은 [Step 11: ApexNAV VLM 통합](11-apexnav-vlm.md)에서 다룹니다.

### 5.2 C++ 플래너 실행

```bash
cd ~/ApexNav_ROS2_wrapper
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch exploration_manager exploration_traj.launch.py
```

> 정상 시작 시 `[exploration_node]`, `[tsp_solver]` 노드가 올라옵니다.
> (`traj_server`는 2026-04-07부터 launch에서 비활성화됨 — `swerve_path_follower.py`가 대체)
> `Starting in REAL WORLD mode` + `Initialization complete` 메시지가 나오면 준비 완료.

### 5.3 물체 탐색 시작

스크립트가 시작되면 **자동으로 360도 회전**하여 초기 SDF 맵을 빌드합니다.
회전이 완료된 후 `target>` 프롬프트가 나타납니다.

> **왜 회전하는가?** real_world 궤적 모드에서는 Habitat의 이산 초기 회전이 없습니다.
> 정면만 보이는 상태에서 frontier를 찾을 수 없어 `open set empty, no path` 에러가 발생합니다.

```bash
# 터미널 12에서:
python3 ~/ms_AIworker/scripts/target_label_publisher.py
# → 자동 360도 회전 (약 13초)
# → [Init] Rotation complete (360 degrees). SDF map ready.

# 대화형 모드:
target> chair
# → /detector/label, /move_base_simple/goal 발행
# → 로봇이 탐색 시작!

# 또는 직접 지정 (회전 후 자동 발행):
python3 ~/ms_AIworker/scripts/target_label_publisher.py chair

# 회전 건너뛰기 (이미 맵이 있는 경우):
python3 ~/ms_AIworker/scripts/target_label_publisher.py --no-rotate
```

### 5.4 RViz에서 확인

RViz Fixed Frame을 **`World`** (대문자 W)로 설정한 후, 다음 토픽을 추가합니다:

**맵 시각화 (PointCloud2)**:
- `/grid_map/occupied` -- 장애물 (빨강)
- `/grid_map/free` -- 자유 공간 (초록)
- `/grid_map/occupied_inflate` -- inflation 영역
- `/grid_map/depth_cloud` -- depth 투영 포인트
- `/grid_map/unknown` -- 미탐사 영역

**경로/궤적 시각화 (Marker/Path)**:
- `/kinoastar/FlatPath` (MarkerArray) -- A* 탐색 경로
- `/kinoastar/FlatTraj` (Path) -- 평탄화된 궤적
- `/trajectory/mincoPath` (Path) -- 최종 최적화 경로
- `/planning_vis/trajectory` (Marker) -- 궤적 시각화
- `/planning_vis/frontier` (Marker) -- frontier 시각화

**카메라 (Image)**:
- `/habitat/camera_rgb` -- RGB 이미지
- `/habitat/camera_depth` -- 정규화된 depth

### 5.5 탐색 과정

IsaacSim은 궤적 모드(`is_real_world=true`)를 사용합니다. FSM 상태:

```
1. [INIT]          FSM 초기화, /detector/confidence_threshold 수신 대기
2. [WAIT_TRIGGER]  /move_base_simple/goal 수신 대기
3. [PLAN_TRAJ]     SDF 맵 빌드, frontier 탐색, 연속 궤적 계획
4. [EXEC_TRAJ]     swerve_path_follower가 PolyTraj 평가 → holonomic /cmd_vel (vx, vy)
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
| `max_vel` | 0.5 | 최대 속도 (m/s). 1.0은 trajectory 추적 불안정. MPC 튜닝 후 0.7까지 가능 |
| `max_acc` | 2.0 | 최대 가속도 (m/s^2) |

> 이 값들은 FFW-SG2의 실제 바퀴 위치(`swerve_controller.py`)에서 계산되었습니다.

### algorithm_traj.launch.py (플래너 파라미터)

`exploration_traj.launch.py`가 내부적으로 `algorithm_traj.launch.py`를 호출합니다.
주요 파라미터는 `algorithm_traj.launch.py`에 있습니다:

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `map_size_x/y` | 80.0 | SDF 맵 크기 (m) |
| `obstacles_inflation` | 0.45 | 장애물 팽창 반경 (m) -- 로봇 반경 |
| `depth_filter_maxdist` | MAX_DEPTH - 0.01 | depth 최대 거리 (m). `config/apexnav_bridge.yaml`의 `max_depth`에서 자동 계산 |
| `local_bound` | 10.0 | ESDF (Euclidean Signed Distance Field) 업데이트 범위 (m). depth 센서 최대 거리와 독립적으로 고정 |
| `perception_utils.max_dist` | 4.0 | perception 라이브러리가 사용하는 최대 거리 (m). depth 범위와 별도로 관리 |
| `free_ray_extrapolation` | 0.95 | depth가 최대 거리를 초과할 때 자유 공간으로 표시하는 거리 배수. 0.95 = max_dist의 95% = 약 4.74m까지 자유 공간으로 마킹 |
| `is_real_world` | true | 궤적 모드 사용 (이산 액션 대신 연속 속도) |
| `cx, cy` | 320, 240 | 카메라 주점 (640x480 기준) |
| `fx, fy` | 245.33, 245.33 | 카메라 초점 거리 (`/zed_mini/camera_info` K 행렬에서 확인) |
| `frame_id` | World | 맵 좌표 프레임 (IsaacSim stage root, 고정) |
| `sensor_pose_topic` | /habitat/camera_pose | 실제 카메라 pose (map_ros depth 투영용). 이름이 `sensor_pose`이지만 `/habitat/sensor_pose`(VLM용)와 다름! |
| `planning_param` | planning_param_ffw.yaml | FFW-SG2 로봇 크기 파라미터 |
| `replan_time` | 1.0 | 궤적 재계획 주기 (초). 낮을수록 새로운 정보에 빠르게 대응 |
| `replan_traj_end_threshold` | 1.5 | 궤적 끝에 도달한 것으로 판단하는 거리 (m). 1.5m 이내면 다음 궤적 계획 시작 |
| `odom_far_threshold` | 1.5 | Odometry 신뢰도 체크 임계값 (m). 최후 예상 위치에서 1.5m 초과 벗어나면 위치 재추정 필요 |
| `min_contain_unknown` | 30 | frontier 셀 최소 개수. 30개 미만 미지 영역은 frontier로 인정하지 않음 |

### config/apexnav_bridge.yaml (브릿지 설정)

**용도**: bridge 노드와 C++ 플래너가 공유하는 설정 파일입니다. 이전에는 bridge 코드 내에 하드코딩되어 있었으나, yaml로 통합하여 `--ros-args --params-file` 옵션으로 런타임에 로드할 수 있습니다.

```yaml
# config/apexnav_bridge.yaml
bridge:
  max_depth: 5.0           # depth 정규화 최대값 (m). bridge가 depth_msg / max_depth로 정규화
  depth_frame_id: "base_link"  # depth 카메라 프레임 ID
  rgb_frame_id: "base_link"    # RGB 카메라 프레임 ID
  odom_topic: "/habitat/odom"  # odometry 토픽 (World 좌표)
  tf_timeout: 2.0          # TF lookup 타임아웃 (초). 초과 시 발행 skip
```

**주요 설정값 설명**:

| 설정 | 기본값 | 설명 |
|------|--------|------|
| `max_depth` | 5.0 | depth 센서 최대 거리 (m). ZED Mini는 약 5m까지 신뢰 가능. 이 값 초과 depth는 1.0으로 정규화되어 C++에서 max-range로 해석됨 |
| `depth_frame_id` | base_link | depth 카메라가 바라보는 프레임. TF `base_link → camera_left` 필요 |
| `rgb_frame_id` | base_link | RGB 카메라 프레임. ZED Mini 기본값 |
| `odom_topic` | /habitat/odom | Odometry 입력 토픽. nav2_bridge에서 발행하는 World 좌표 odometry |
| `tf_timeout` | 2.0 | TF lookup이 2초 이상 걸리면 "timeout" 경고 후 skip. IsaacSim은 보통 0.5초 이내 |

**실행 방법**:

```bash
# yaml 파일과 함께 bridge 실행
python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py \
  --ros-args --params-file ~/ms_AIworker/config/apexnav_bridge.yaml

# 또는 명령행 오버라이드 (yaml보다 우선)
python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py \
  --ros-args \
  --params-file ~/ms_AIworker/config/apexnav_bridge.yaml \
  -p bridge.max_depth:=4.5
```

**C++ 플래너와의 연동**:

`algorithm_traj.launch.py`는 다음 값을 yaml에서 읽어 자동 계산합니다:

```python
# algorithm_traj.launch.py
max_depth = get_parameter("bridge.max_depth")  # yaml에서 읽음
depth_filter_maxdist = max_depth - 0.01       # C++ map_ros에 전달
free_ray_extrapolation = 0.95                 # 최대 거리의 95%까지 자유 공간 마킹
```

이렇게 하면 bridge와 C++ 플래너가 같은 depth 범위를 사용하여 불일치로 인한 버그를 방지합니다.

---

## 환경 프로파일 (Environment Profiles)

ApexNAV는 환경에 따라 다른 파라미터 프로파일이 필요합니다. 기본 설정은 가정집/실내 환경에 최적화되어 있으며, 공장/창고 같은 대규모 환경에는 별도 프로파일을 사용합니다.

### 사용법

```bash
# 가정집 환경 (기본)
ros2 launch exploration_manager exploration_traj.launch.py

# 공장/창고 환경
ros2 launch exploration_manager exploration_traj_factory.launch.py
```

### 파라미터 비교

| 파라미터 | 가정집 (기본) | 공장/창고 | 이유 |
|---------|-------------|----------|------|
| `map_size` | 80×80m | 200×200m | 넓은 공간 커버 |
| `resolution` | 0.05m | 0.15m | 메모리 절약 (25M cells) |
| `local_bound` | 10m | 25m | ESDF 업데이트 범위 확대 |
| `obstacles_inflation` | 0.45m | 0.50m | 거친 resolution 보정 |
| `perception max_dist` | 4.0m | 4.8m | depth 5m 최대 활용 |
| `frontier cluster_min` | 5 | 3 | 거친 resolution에서 작은 클러스터 감지 |
| `frontier cluster_size` | 0.65m | 1.5m | 넓은 프론티어 병합 |
| `min_contain_unknown` | 30 | 8 | resolution 비례 축소 |
| `astar resolution` | 0.1m | 0.4m | 장거리 탐색 속도 |
| `step_arc` | 0.2m | 0.5m | KinoAstar 탐색 효율 |
| `max_search_time` | 2.0s | 5.0s | 장거리 경로 계획 여유 |
| `free_ray_extrapolation` | 0.95 | 1.5 | 열린 공간 빠른 매핑 |
| `local_target_distance` | 4.0m | 10.0m | 긴 trajectory 계획 |

> **`free_ray_extrapolation` 설명**: depth가 최대 거리(`depth_filter_maxdist * 0.96`)를 초과할 때 free 공간으로 표시하는 거리 배수입니다.
> - 가정집(0.95): 최대 거리의 95% → 약 4.74m까지 free 마킹
> - 공장(1.5): 최대 거리의 150% → 약 7.5m까지 free 마킹 (넓은 통로에서 효과적)

### 주의사항

> **공장 → 가정집 복귀 시**: 반드시 기본 launch 파일(`exploration_traj.launch.py`)로 변경하여 실행하세요.

- 공장 모드의 `resolution 0.15m`는 좁은 문(0.8m) 통과가 어려울 수 있음
- `free_ray_extrapolation=1.5`는 벽이 많은 환경에서 벽 뒤 유령 free 공간 생성 위험

---

## Swerve 로봇용 패치 (2026-04-07 ~ 2026-04-09)

World2 개방 환경에서 자율 주행이 조기에 `No passable frontier` 종료되던 문제를 해결하고, trajectory 추적 안정성을 개선하기 위해 다음 패치를 적용했습니다.

### Step 0: swerve_path_follower.py 대폭 개선 (2026-04-09)

**파일**: `scripts/swerve_path_follower.py`

#### 0.1 Odometry 소스 변경: `/odom` → `/habitat/odom`

**변경 사항**: odometry 구독을 `/habitat/odom`으로 변경 (World 고정 좌표 사용)

- **이전**: `/odom` (nav2_bridge 발행, 문제: sim time / wall time 혼합, 노이즈)
- **이후**: `/habitat/odom` (IsaacSim 센서에서 직접 발행, 신뢰성 높음)
- **왜**: `/odom`은 IsaacSim Play 타임스탬프와 wall clock이 섞여 있어 TF lookup과 불일치

**확인 방법**:
```bash
ros2 topic echo /habitat/odom | head -20
# header.stamp.sec/nsec 값이 simulation time과 일치해야 함
```

#### 0.2 Feedforward + P 제어 추가

**알고리즘**: trajectory에서 선속도(vx, vy) 직접 계산 + 위치 P 제어

```
feedforward_vel = d/dt(trajectory_polynomial)  # 다항식 미분으로 속도 계산
error_vel = K_p * (desired_pos - current_pos)  # 위치 오차 피드백
final_vel = feedforward_vel + error_vel        # 합산
```

**효과**:
- 이전: lookahead pure pursuit만 사용 → trajectory가 가파를 때 추적 지연
- 이후: feedforward로 속도를 미리 계산 → 부드러운 추적 + 더 빠른 응답

**파라미터** (코드 내):
- `K_p_vx, K_p_vy`: 위치 피드백 게인 (기본 0.5)
- `trajectory.differentiate()`: 다항식 7차 미분 (PolyTraj septic)

#### 0.3 Yaw Tracking (방향 제어)

**목표**: depth 카메라가 항상 로봇의 진행 방향을 보도록 회전

```
desired_yaw = atan2(vy_desired, vx_desired)
yaw_error = desired_yaw - current_yaw
if |yaw_error| > 80°:
    제자리 회전 (vx=0, vy=0, angular_z=sign(yaw_error) * max_angular_vel)
else:
    회전하며 이동 (angular.z = K_angular * yaw_error)
```

**임계값 설명**:
- 80° 이상 차이 = 거의 역방향이므로 먼저 회전
- 80° 미만 = 회전하며 동시에 이동 (더 빠름)

**왜 필요한가**?
- ApexNAV의 depth 프로세싱이 forward-looking 카메라를 가정
- 카메라가 옆을 보면 frontier 감지 성능 저하

#### 0.4 Trajectory 전환 부드러움 처리

**변경**: trajectory 체인지 포인트에서의 끊김 제거

- **이전**: 새 trajectory 수신 시 즉시 시작 → 속도 급변 가능
- **이후**: 이전 trajectory의 끝 상태(위치, 속도)에서 매끄럽게 연결

**구현**:
```python
if new_trajectory_received:
    # 이전 궤적의 마지막 시점 상태 읽음
    prev_state = old_trajectory.evaluate(old_traj_duration)
    # 새 궤적과 매끄럽게 연결 (위치/속도 일치)
    blend_trajectory(prev_state, new_trajectory)
```

#### 0.5 정지 조건 강화

**정지 판정 (trajectory 끝 도달)**:
- 조건 1: trajectory 끝 도달 (끝점 0.2m 이내) **AND** 0.5초 동안 명령 미수신
- 조건 2: 5초 동안 trajectory 명령 미수신 (crash 방어)

**왜 필요한가**?
- `traj_server` crash 시 무한 대기를 방지
- `/planning/trajectory` topic이 끊어진 것을 감지

**로그**:
```
[swerve_path_follower] Trajectory complete + no command for 0.5s → STOP
[swerve_path_follower] Command timeout 5s → STOP (crash?)
```

#### 0.6 Sim Time / Wall Time 문제 해결

**변경**: 모든 타임스탬프를 wall clock 기준으로 통일

- **이전**: IsaacSim `sim_time` + `/odom` `wall_time` 혼합 → TF lookup 오류
- **이후**: 항상 `rospy.Time.now()` (wall clock) 사용

**효과**:
```
TF lookup error: "time_source mismatch [1 != 2]" → 해결됨
```

#### 검증 결과 (2026-04-09)

| 항목 | 결과 |
|------|------|
| Trajectory 추적 | 20+ 회 연속 완료 |
| Feedforward + P 제어 | 안정적 수렴 (Kp=0.5) |
| Yaw tracking | depth 카메라 항상 전방 유지 |
| Trajectory 전환 | 끊김 없음 |
| 정지 안정성 | crash 감지 성공 |

---

### Step 1: 카메라 pose NaN 가드

**파일**: `scripts/isaacsim_apexnav_bridge.py`

- `_get_camera_pose_from_tf()`: quaternion `finite` 체크 + norm `[0.99, 1.01]` 범위 검증. 실패 시 발행 skip, throttled warn 로그
- `_make_habitat_sensor_pose()`: yaw + robot_pos finite 체크
- **증상**: map_ros의 `euler_from_quaternion()` NaN 에러
- **검증**: 10분 가동 후 `NaN` 에러 로그 0건 확인
- **롤백**: `git revert` 해당 커밋

### Step 1b: target_label_publisher.py 수정 (2026-04-09)

**파일**: `scripts/target_label_publisher.py`

**변경 사항**: 초기 회전 시 yaw 참고값을 odometry에서 TF로 변경

```python
# 이전 (odometry 기반 — 노이즈 있음)
initial_yaw = msg.pose.pose.orientation  # /odom의 quaternion → Euler

# 이후 (TF 기반 — ground-truth)
tf_world_to_base = tf_buffer.lookup_transform("World", "base_link", ...)
q = tf_world_to_base.transform.rotation
initial_yaw = euler_from_quaternion(q)  # 정확한 물리 yaw
```

**동작**:

```
초기 회전 시 정확히 360° 한 바퀴만 회전
- 시작 yaw: tf lookup으로 정확히 결정
- 목표 yaw: start_yaw + 360°
- 종료: 360° 완전히 도달했을 때만 STOP
```

**왜 필요한가**?
- odometry는 노이즈가 있어 초기 yaw 추정 부정확
- 회전 중간에 `/odom` 업데이트가 부정확하면 회전 범위 변함
- TF는 IsaacSim의 실제 pose 반영 (ground-truth)

**확인 방법**:

```bash
# 회전 중 로그 확인
ros2 run rqt_graph rqt_graph  # TF가 꾸준히 업데이트되는지 확인
# /World → /world → /base_link 체인이 보여야 함
```

---

### Step 2: Footprint 실측값 복원

**파일**: `~/ApexNav_ROS2_wrapper/.../planning_param_ffw.yaml`

```yaml
kino_astar:
  length: 0.56        # 변경: 0.70 → 0.56 (사용자 실측, 2026-04-01)
  width: 0.51         # 변경: 0.65 → 0.51
  wheel_base: 0.43    # 유지
```

**파일 상단 주석**:
```yaml
# WARNING: Footprint locked to measured dimensions (2026-04-01).
# Do NOT auto-tune. Contact team before adjustment.
```

- **증상**: 부풀린 footprint로 인한 `isCollisionPosYaw` 과다 호출
- **검증**: `git diff` 확인, `colcon build` 후 launch
- **롤백**: 해당 yaml 파일 git revert

### Step 3a: filter_min_height 복원

**파일**: `algorithm_traj.launch.py:78`

```python
DeclareLaunchArgument('filter_min_height', default_value='0.5',  # 변경: 0.05 → 0.5
```

- **증상**: 로봇 바닥 점이 OCCUPIED로 잘못 분류
- **검증**: RViz OccupancyGrid에서 로봇 주변 가짜 OCCUPIED 덩어리 사라짐
- **롤백**: launch 파일 git revert

### Step 3b: obstacles_inflation 복원

**파일**: `algorithm_traj.launch.py:59`

```python
DeclareLaunchArgument('obstacles_inflation', default_value='0.45',  # 변경: 0.1 → 0.45
```

- **검증**: RViz inflation layer 두께 확인 (footprint + 0.4~0.45m)
- **롤백**: launch 파일 git revert

### Step 4a: checkCollision 방어적 주석

**파일**: `kino_astar.cpp:705-712`

```cpp
int state = map_->getOccupancy(pos);
// SDFMap2D enum: UNKNOWN=0, FREE=1, OCCUPIED=2, out-of-grid=-1
```

- 동작 변화 없음 (의도 문서화)
- **롤백**: 함수 단위 git revert

### Step 4b: isCollisionPosYaw Swerve 모드

**파일**: `kino_astar.cpp:641-710`, `kino_astar.h`, `planning_param_ffw.yaml`

```yaml
kino_astar:
  is_swerve: true     # NEW: swerve 모드 활성화
```

- **true일 때**: axis-aligned 박스 (회전 고정, `length`, `width` max 값 사용)
- **동작**: lateral step에서 끊기지 않는 trajectory 생성
- **검증**: RViz에서 trajectory 연속성 확인, 5분 가동 중 `isCollisionPosYaw occ` ≤ 5건
- **롤백**: yaml 플래그 false로 변경 또는 git revert

### Step 5: algorithm_traj.launch.py 파라미터 최적화 (2026-04-09)

**파일**: `~/ApexNav_ROS2_wrapper/.../algorithm_traj.launch.py`

다음 파라미터들을 조정하여 trajectory 안정성과 exploration 성능을 개선했습니다:

#### 5.1 재계획 및 추적 안정성

```python
# 이전 설정
replan_time = 2.0              # 재계획 주기 2초 → 너무 김
replan_traj_end_threshold = 1.0  # 궤적 끝 판정 거리 1.0m → 너무 가까움

# 신규 설정
replan_time = 1.0              # 재계획 주기 1초 → 더 빠른 대응
replan_traj_end_threshold = 1.5  # 궤적 끝 판정 거리 1.5m → 미리 다음 궤적 준비
```

**설정값 의미**:
- `replan_time=1.0`: 1초마다 새로운 frontier 정보 반영하여 궤적 재계획
- `replan_traj_end_threshold=1.5`: trajectory 끝점까지 1.5m 남았을 때 다음 계획 시작 (미리 준비해서 끊김 방지)

#### 5.2 Odom 신뢰도

```python
# exploration_fsm_traj.cpp에서 사용
odom_far_threshold = 1.5       # odometry 벗어남 거리 (이전 0.6)
```

**설정값 의미**:
- IsaacSim의 odom은 매우 정확하므로 1.5m까지 허용
- 실제 로봇에서는 0.6m 정도로 낮춤 (odometry drift 고려)

#### 5.3 Free Ray 외삽

```python
free_ray_extrapolation = 0.95  # depth 최대 거리의 95% 지점부터 자유 공간
```

**설명**:
- depth 센서가 5.0m까지 측정하면 → 5.0 * 0.95 = 4.75m 이후는 "측정되지 않음 = 자유"로 해석
- 개방 환경에서 원거리 frontier를 빠르게 감지

#### 5.4 Frontier 최소 조건

```python
min_contain_unknown = 30       # frontier의 미지 셀 최소 개수
```

**설명**:
- 30개 미만의 작은 frontier는 무시
- IsaacSim의 depth 성능이 좋으므로 작은 미지 영역도 빠르게 매핑 → 작은 frontier 필터링 가능

#### 검증 (2026-04-09)

| 파라미터 변경 | 기대 효과 | 결과 |
|------------|---------|------|
| `replan_time`: 2.0 → 1.0 | 새 frontier 빠른 반영 | trajectory 진동 없음 |
| `replan_traj_end_threshold`: 1.0 → 1.5 | 끝점 미리 감지 | 끊김 없는 전환 |
| `odom_far_threshold`: 0.6 → 1.5 | IsaacSim 높은 정확도 활용 | localization 에러 0 |
| `free_ray_extrapolation`: 0.95 유지 | frontier 빠른 감지 | 원거리 탐색 안정 |
| `min_contain_unknown`: 30 유지 | 노이즈 frontier 필터링 | 불필요한 회전 감소 |

---

### 빌드 및 실행

```bash
cd ~/ApexNav_ROS2_wrapper
colcon build --packages-select path_searching exploration_manager trajectory_manager --symlink-install
source install/setup.bash
ros2 launch exploration_manager exploration_traj.launch.py
```

### Play 검증 절차 (Evidence Matrix)

IsaacSim Play 후 다음 기준으로 통과 판정:

| 항목 | 기준 | 측정 방법 |
|------|------|---------|
| Trajectory 생성 | ≥ 20회 / 5분 | C++ 콘솔 `traj_count` 로그 또는 `/planning_vis/trajectory` marker 카운트 |
| 충돌 에러 | ≤ 5건 / 5분 | `grep "isCollisionPosYaw occ" launch 로그 \| wc -l` |
| 조기 종료 | ≥ 180s 이후 | `grep "No passable frontier"` 발생 시각 기록 |
| 서버 안정성 | crash 0건 또는 격리됨 | `traj_server` 에러 로그 모니터링 |

**로그 수집 명령**:
```bash
# Terminal: ros2 launch exploration_manager exploration_traj.launch.py 2>&1 | tee /tmp/apexnav_run.log &
# 5분 대기
sleep 300

# 분석
grep "isCollisionPosYaw occ" /tmp/apexnav_run.log | wc -l
grep "No passable frontier" /tmp/apexnav_run.log | head -1
```

---

## [7] Troubleshooting

### ApexNAV Bridge에서 아무 토픽도 안 나옴 (TF lookup 실패)

- bridge가 `World → base_link` TF lookup에 실패하면 **조용히 무시**합니다 (에러 메시지 없음)
- 확인 방법:
  ```bash
  ros2 run tf2_tools view_frames
  # World → world → base_link 체인이 있어야 함
  ```
- TF가 안 나오면:
  1. IsaacSim이 **Play** 상태인지 확인 (Pause/Stop이면 TF 안 나옴)
  2. `nav2_bridge.py`가 실행 중인지 확인 (`odom → World` static TF 필요)
  3. IsaacSim TF Publisher Action Graph가 설정되어 있는지 확인 ([Step 4](04-publish-tf.md) 참고)

### conda 환경 충돌 (ImportError, Python 버전 불일치)

- conda base(Python 3.13)가 활성화된 상태에서 ROS2 스크립트를 실행하면 충돌합니다
- **반드시 `conda deactivate`** 후 `source ros2-bridge-env.sh`로 실행하세요
- 증상: `ImportError: libpython3.12.so` 또는 `rclpy` import 실패

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

### /cmd_vel이 0으로만 나옴

- C++ 플래너가 실행 중인지 확인
- `/habitat/odom`, `/habitat/camera_depth` 토픽이 들어오는지 확인
- `traj_server` 로그에 에러 없는지 확인

### 로봇이 경로를 이탈함 (trajectory 추적 불안정)

- `planning_param_ffw.yaml`의 `max_vel`이 너무 높으면 MPC가 궤적을 추적하지 못합니다
- 기본값 0.5로 설정되어 있지만, 수정한 적이 있다면 확인하세요:
  - `planning_param_ffw.yaml`: root, `kino_astar`, `optimizer` 3곳 모두 동일해야 함
  - `exploration_traj.launch.py`: `max_correction_vel`도 같은 값이어야 함
- MPC 튜닝(`control_param.yaml`의 `matrix_q` 속도 가중치)을 올리면 0.7까지 가능

### RViz에서 Depth 이미지가 깨져 보임

32FC1 depth 이미지가 세로 줄무늬로 보이는 경우:
1. Image display -> **Normalize Range** 체크 해제
2. **Min Value**: 0.0, **Max Value**: 1.0 (정규화된 값이므로)

또는 `rqt_image_view`로 확인:
```bash
ros2 run rqt_image_view rqt_image_view /habitat/camera_depth
```

---

**Status**: Phase 2 완료 + Swerve 최적화 (2026-04-09)
- Phase 1: 센서 + 브릿지 연결 (depth, rgb, odom -> /habitat/* 토픽)
- Phase 2: ApexNAV 플래너 연결 (SDF 맵, frontier 탐색, 자율 이동)
- **2026-04-09**: swerve_path_follower feedforward + P 제어 추가, /habitat/odom 전환, TF 기반 yaw 추정
- **2026-04-09**: algorithm_traj.launch.py 파라미터 최적화 (replan_time 2.0→1.0, odom_far_threshold 0.6→1.5)
- **2026-04-09**: config/apexnav_bridge.yaml 신규 생성 (depth 설정 통합)
- **TODO**: RViz 시각화 설정 -- 일부 토픽(frontier, trajectory 등)이 RViz에서 안 보이는 문제 확인 필요

---

**이전**: [Step 9: ApexNAV 브릿지](09-apexnav-bridge.md) | **다음**: [Step 11: ApexNAV VLM 통합](11-apexnav-vlm.md)
