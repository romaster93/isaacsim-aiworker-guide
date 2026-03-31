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
| **4** | `conda deactivate` + `source ros2-bridge-env.sh` | ApexNAV Bridge | `python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py` |

> **확인**: `ros2 topic list | grep habitat` -> `/habitat/odom`, `/habitat/camera_rgb` 등이 보여야 합니다.

**Phase C -- ApexNAV 플래너 실행**

> **순서 주의**: `target_label_publisher.py`를 **먼저** 실행한 후 C++ 플래너를 실행하세요.
> 플래너가 먼저 뜨면 `/detector/confidence_threshold`가 없어서 `[Real] No odom || No target confidence threshold` 경고가 반복됩니다.

| 터미널 | 환경 | 내용 | 명령어 |
|--------|------|------|--------|
| **11** | `conda deactivate` + `source ros2-bridge-env.sh` | RViz | `rviz2 --ros-args -p use_sim_time:=true` |
| **12** | `conda deactivate` + `source ros2-bridge-env.sh` | 물체 명령 (먼저!) | `python3 ~/ms_AIworker/scripts/target_label_publisher.py` |
| **9** | `conda deactivate` + `source ros2-bridge-env.sh` + `source ~/ApexNav_ROS2_wrapper/install/setup.bash` | C++ 플래너 | `ros2 launch exploration_manager exploration_traj.launch.py` |

> **확인**: 터미널 9에서 `Exploration FSM initialized` 메시지가 나오면 준비 완료.

> **VLM 통합** (Phase B + VLM 노드)은 [Step 11: ApexNAV VLM 통합](11-apexnav-vlm.md)에서 다룹니다.

### 5.2 C++ 플래너 실행

```bash
cd ~/ApexNav_ROS2_wrapper
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch exploration_manager exploration_traj.launch.py
```

> 정상 시작 시 `[exploration_node]`, `[traj_server]`, `[tsp_solver]` 노드가 올라옵니다.
> `Starting in REAL WORLD mode` + `Initialization complete` 메시지가 나오면 준비 완료.

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

### algorithm_traj.launch.py (플래너 파라미터)

`exploration_traj.launch.py`가 내부적으로 `algorithm_traj.launch.py`를 호출합니다.
주요 파라미터는 `algorithm_traj.launch.py`에 있습니다:

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `map_size_x/y` | 20.0 | SDF 맵 크기 (m) |
| `obstacles_inflation` | 0.45 | 장애물 팽창 반경 (m) -- 로봇 반경 |
| `depth_filter_maxdist` | 4.99 | depth 최대 거리 (m) |
| `is_real_world` | true | 궤적 모드 사용 (이산 액션 대신 연속 속도) |
| `cx, cy` | 320, 240 | 카메라 주점 (640x480 기준) |
| `fx, fy` | 245.33, 245.33 | 카메라 초점 거리 (`/zed_mini/camera_info` K 행렬에서 확인) |
| `frame_id` | World | 맵 좌표 프레임 (IsaacSim stage root, 고정) |
| `sensor_pose_topic` | /habitat/camera_pose | 실제 카메라 pose (map_ros depth 투영용) |
| `planning_param` | planning_param_ffw.yaml | FFW-SG2 로봇 크기 파라미터 |

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

### /cmd_vel이 0으로만 나옴

- C++ 플래너가 실행 중인지 확인
- `/habitat/odom`, `/habitat/camera_depth` 토픽이 들어오는지 확인
- `traj_server` 로그에 에러 없는지 확인

### RViz에서 Depth 이미지가 깨져 보임

32FC1 depth 이미지가 세로 줄무늬로 보이는 경우:
1. Image display -> **Normalize Range** 체크 해제
2. **Min Value**: 0.0, **Max Value**: 1.0 (정규화된 값이므로)

또는 `rqt_image_view`로 확인:
```bash
ros2 run rqt_image_view rqt_image_view /habitat/camera_depth
```

---

**Status**: Phase 2 완료 (SDF 맵 빌드 + 자율 주행 성공)
- Phase 1: 센서 + 브릿지 연결 (depth, rgb, odom -> /habitat/* 토픽)
- Phase 2: ApexNAV 플래너 연결 (SDF 맵, frontier 탐색, 자율 이동)
- **TODO**: RViz 시각화 설정 -- 일부 토픽(frontier, trajectory 등)이 RViz에서 안 보이는 문제 확인 필요

---

**이전**: [Step 9: ApexNAV 브릿지](09-apexnav-bridge.md) | **다음**: [Step 11: ApexNAV VLM 통합](11-apexnav-vlm.md)
