# Step 7: Navigation System (Nav2)

## Overview

이 가이드에서는 IsaacSim 상의 FFW-SG2 로봇에 **Nav2(Navigation2)** 자율 주행 시스템을 연동합니다.

Nav2란? ROS2의 표준 자율 주행 프레임워크입니다. 로봇이 **스스로 지도를 만들고**(SLAM),
**목표 지점까지 장애물을 피해 이동**할 수 있게 해줍니다.

이 가이드를 끝내면:
- IsaacSim 환경의 지도를 SLAM으로 만들 수 있음
- RViz에서 목표점을 클릭하면 로봇이 자동으로 이동
- Nav2의 경로 계획 → 장애물 회피 → 도착 과정을 이해

> **참고**: 이 가이드는 ROBOTIS ai_worker의
> [Navigation 가이드](https://ai.robotis.com/ai_worker/operation_navigation_ai_worker.html)를
> IsaacSim 환경에 맞게 재구성한 것입니다.

### 가이드 구조

| 섹션 | 내용 | 할 일 |
|------|------|-------|
| [1] 개념 이해 | Nav2가 뭔지, 뭘 필요로 하는지 | 읽기 |
| [2] 패키지 설치 | Nav2, SLAM Toolbox 설치 | **터미널에서 실행** |
| [3] 브릿지 설정 | IsaacSim ↔ Nav2 연결 | **터미널에서 실행** |
| [4] SLAM | 지도 만들기 (텔레옵 / 자동 매핑) | **IsaacSim + 터미널** |
| [5] 자율 주행 | 목표점까지 자동 이동 | **IsaacSim + RViz** |
| [6] 설정 파일 | 파라미터 설명 | 필요할 때 참고 |
| [7] 트러블슈팅 | 안 될 때 해결법 | 필요할 때 참고 |

## Prerequisites
- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 완료 (Step 2)
- [x] 센서 구성 완료 — 특히 **2D LiDAR** (Step 3)
- [x] TF 발행 완료 (Step 4)
- [x] 관절 제어 완료 (Step 5)
- [x] Swerve Drive 동작 확인 (Step 6)
- [x] ROS2 Jazzy 환경 (`conda deactivate` + `source ros2-bridge-env.sh`)

---

## [1] Nav2 개념 이해

### Nav2가 하는 일

```
사용자: "저기로 가!"  →  Nav2: 경로 계획 → 장애물 회피 → 도착!
                            ↑
                      지도 + 현재 위치 + 센서 데이터
```

Nav2는 3가지를 조합해서 자율 주행합니다:
1. **지도 (Map)** — 환경이 어떻게 생겼는지 (SLAM으로 만듦)
2. **위치 추정 (Localization)** — 지금 로봇이 어디에 있는지 (AMCL)
3. **센서 (LiDAR)** — 실시간 장애물 감지

### Nav2가 필요로 하는 것

Nav2를 돌리려면 로봇이 이 토픽/TF를 제공해야 합니다:

| 필요 요소 | 토픽/TF | 현재 상태 | 해결 방법 |
|-----------|---------|-----------|-----------|
| Odometry | `/odom` + `odom → base_link` TF | **없음** | `swerve_controller.py`(odometry) + `nav2_bridge.py`(TF)로 해결 |
| LiDAR | `/scan` (LaserScan) | `/laser_scan_left`, `/laser_scan_right` 있음 | `laser_merger.py`로 해결 |
| TF Tree | `base_link → 센서/링크` | **완료** (Step 4) | — |
| Map → Odom | `map → odom` TF | — | AMCL/SLAM이 자동 발행 |

### TF Tree 구조

Nav2가 기대하는 TF 체인:

```
map → odom → base_link → (모든 센서/링크)
 ↑      ↑        ↑              ↑
AMCL  Bridge  IsaacSim TF    IsaacSim TF
```

우리의 구현:

```
map → odom → World → world → base_link → head_link1 → ...
 ↑      ↑       ↑       ↑         ↑
AMCL  static  static  IsaacSim  IsaacSim
      (bridge) (bridge) TF       TF Publisher
```

`nav2_bridge.py`가 `odom → World` 정적 변환을 발행해서,
IsaacSim의 `World → world → base_link`과 연결합니다.
시뮬레이션에서는 `World = odom`이므로 동일 좌표계입니다.

> **주의**: IsaacSim은 Stage root로 `World` (대문자 W)를 사용합니다.
> TF 체인은 `map → odom → World → world → base_link` 순서입니다.
> `world` (소문자)는 IsaacSim 내부 TF이며, `World` (대문자)는 Stage root입니다.

---

## [2] Nav2 패키지 설치

```bash
# conda 비활성화 (ROS2 Jazzy와 충돌 방지)
conda deactivate

# Nav2 핵심 패키지
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox

# tf2 관련 (nav2_bridge.py에서 사용)
sudo apt install -y \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs
```

> **확인**: 설치 후 `ros2 pkg list | grep nav2` 실행 — nav2 패키지들이 보여야 합니다.

---

## [3] IsaacSim ↔ Nav2 브릿지 설정

IsaacSim은 게임 엔진 기반이고, Nav2는 ROS2 기반입니다.
둘을 연결하려면 **3가지를 브릿지**해야 합니다.

### 3.0 Joint State Publisher 설정 (IsaacSim)

`swerve_controller.py`의 FK odometry는 `/joint_states` 토픽이 필요합니다.
IsaacSim에서 Joint State Publisher를 추가합니다:

1. `Tools` → `Robotics` → `ROS2 OmniGraphs` → **ros jointstate** 선택
2. **Publisher만 체크** (Subscriber는 체크 해제)
3. Articulation Root: `/ffw_sg2_follower` 하위의 **`world`** prim 선택
4. OK 클릭

> **Articulation이란?** 관절로 연결된 물리 바디들의 묶음입니다.
> Articulation Root는 "이 prim 아래 모든 joint/link를 하나의 로봇으로 취급하라"는 표시입니다.
> FFW-SG2에서는 `world` prim이 Articulation Root입니다.

동작 확인 (Play 후):
```bash
ros2 topic echo /joint_states --once --field name
```
바퀴 joint 이름(`left_wheel_steer`, `left_wheel_drive` 등)이 나오면 성공입니다.

### 3.1 Odometry + TF 브릿지

Step 6에서 `swerve_controller.py`는 `/cmd_vel` → 바퀴 명령(IK)과 `/joint_states` → `/odom`(FK) 두 가지를 처리합니다.
`nav2_bridge.py`는 Nav2 TF 체인을 완성하는 정적 변환만 발행합니다.

`scripts/nav2_bridge.py`가 하는 일:
- **Static TF 발행**: `odom → World` (동일 좌표계, identity transform)
- IsaacSim의 `World → world → base_link` TF와 연결하여 Nav2가 이해하는 TF 체인 완성

`scripts/swerve_controller.py`가 하는 일:
- **IK (역운동학)**: `/cmd_vel` → `/isaac_sim/joint_commands` (바퀴 제어)
- **FK (정운동학)**: `/joint_states` → `/odom` (엔코더 기반 odometry 발행)

> **FK Odometry란?**
> 역운동학(IK)의 반대입니다. IK는 "로봇 속도 → 바퀴 명령"이고,
> FK는 "바퀴 상태 → 로봇 속도"입니다.
> 각 바퀴의 steer 각도와 drive 각속도를 읽어서
> 최소제곱법(SVD)으로 로봇의 속도(vx, vy, ω)를 추정합니다.
> ROBOTIS `ffw_swerve_drive_controller`의 `odometry.cpp`와 동일한 방식입니다.
>
> 실제 로봇처럼 엔코더 기반 odometry이므로 시간이 지나면 **드리프트**가 발생합니다.
> 이 드리프트를 AMCL이 LiDAR 스캔으로 보정합니다 (sim-to-real).

nav2_bridge.py 실행 후 기대 출력:

```
[INFO] [nav2_bridge]: Nav2 Bridge started
  - Static TF: odom → World (identity)
```

swerve_controller.py 실행 후 기대 출력:

```
[INFO] [swerve_controller]: Swerve Drive Controller started
  - IK: /cmd_vel → /isaac_sim/joint_commands
  - FK: /joint_states → /odom (encoder-based odometry)
```

> **대안: Action Graph로 Odometry 발행**
>
> IsaacSim 5.1.0에는 `Isaac Compute Odometry` + `ROS2 Publish Odometry` OmniGraph 노드가 있습니다.
> 이 노드들을 Action Graph에 추가하면 `swerve_controller.py`의 FK odometry를 대체하여 `/odom`을 발행할 수 있습니다.
> 단, Action Graph GUI 프리징 이슈가 있어서 (Step 5 참고) 이 가이드에서는 Python 스크립트 방식을 사용합니다.
>
> Action Graph 방식이 궁금하다면:
> - `Isaac Compute Odometry` — Chassis Prim에서 속도/가속도 계산
> - `ROS2 Publish Odometry` — `/odom` 토픽 발행 (odomFrameId: "odom", chassisFrameId: "base_link")
> - `ROS2 Publish Raw Transform Tree` — `parentFrameId`를 "odom"으로 설정 가능

### 3.2 LiDAR 토픽 설정

Step 3에서 2D LiDAR를 설정했습니다:
- 왼쪽: `/laser_scan_left`
- 오른쪽: `/laser_scan_right`

Nav2는 **`/scan`** 토픽을 기대합니다. `laser_merger.py`로 양쪽 LiDAR를 합쳐서 발행합니다:

```bash
# 양쪽 LiDAR를 합쳐서 /scan으로 발행
python3 ~/ms_AIworker/scripts/laser_merger.py
```

> **참고**: ROBOTIS 실제 로봇은 `dual_laser_merger` 패키지로 양쪽 LiDAR를 병합합니다.
> `laser_merger.py`는 이와 동일한 역할을 합니다.

### 3.3 전체 실행 순서

모든 것을 실행하기 전에, 각 터미널에서 ROS2 환경을 설정합니다:

```bash
# 모든 터미널에서 먼저 실행
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
```

| 터미널 | 실행 내용 | 명령어 |
|--------|----------|--------|
| **1** | IsaacSim | `conda activate isaac_sim && isaacsim` → Play |
| **2** | Swerve Controller + Odometry | `python3 ~/ms_AIworker/scripts/swerve_controller.py` |
| **3** | Nav2 TF Bridge | `python3 ~/ms_AIworker/scripts/nav2_bridge.py` |
| **4** | LiDAR Merger | `python3 ~/ms_AIworker/scripts/laser_merger.py` |

> **동작 확인** (새 터미널에서):
> ```bash
> # odom 토픽 확인
> ros2 topic echo /odom --once
>
> # scan 토픽 확인
> ros2 topic echo /scan --once
>
> # TF 트리 확인 (odom → World → world → base_link 체인이 보여야 함)
> ros2 run tf2_tools view_frames
> ```

---

## [4] SLAM — 지도 만들기

SLAM(Simultaneous Localization and Mapping)으로 IsaacSim 환경의 지도를 만듭니다.

### 4.1 IsaacSim 환경 준비

SLAM을 하려면 LiDAR가 스캔할 수 있는 **벽이나 장애물이 있는 환경**이 필요합니다.

> 아직 환경을 구성하지 않았다면:
> 1. IsaacSim에서 `File → New from Stage Template → Default Stage`
> 2. URDF 로봇을 가져오기 (Step 2)
> 3. `Create → Shapes` → Cube/Cylinder를 배치하여 벽과 장애물 만들기
> 4. 또는 IsaacSim의 샘플 환경(warehouse 등)을 사용

### 4.2 SLAM 실행

위의 터미널 1~4가 실행 중인 상태에서:

**터미널 5 — SLAM Toolbox 실행:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=$HOME/ms_AIworker/config/slam_params.yaml \
  use_sim_time:=true
```

SLAM Toolbox 정상 시작 시 아래와 같은 로그가 나옵니다:

```
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[slam_toolbox]: Loaded params from ...slam_params.yaml
[slam_toolbox]: Message Filter dropping message: frame 'base_link' ...
(첫 스캔 수신 후 사라짐)
[slam_toolbox]: Registering sensor: [Custom Described Lidar]
```

**터미널 6 — RViz 실행:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

rviz2 --ros-args -p use_sim_time:=true
```

> **주의**: `use_sim_time` 없이 RViz를 실행하면 TF 타이밍 에러가 발생합니다.

### 4.3 RViz 설정

RViz가 열리면 시각화를 설정합니다:

1. **Fixed Frame** 변경: 좌측 Displays 패널 → `Fixed Frame` → **`map`** 으로 변경

2. **지도 추가**: `Add` → `By topic` → `/map` → `Map` 선택
   - 지도가 보이지 않으면: Map 항목에서 `Durability Policy` → **Transient Local**

3. **LiDAR 추가**: `Add` → `By topic` → `/scan` → `LaserScan` 선택
   - 빨간 점이 안 보이면: LaserScan 항목에서 `Reliability Policy` → **Best Effort**
   - `Size` → `0.05` (점 크기)

4. **로봇 모델 추가** (선택): `Add` → `By display type` → `TF` 선택
   - TF 트리가 시각화됩니다

5. **RViz 설정 저장**: `File → Save Config As` → `~/ms_AIworker/config/nav2.rviz`

### 4.4 텔레옵으로 매핑

로봇을 이동시키면서 LiDAR가 환경을 스캔하여 지도를 만듭니다.

**터미널 7 — 키보드 텔레옵:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

# ROS2 기본 텔레옵 사용
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> **조작법**:
> - `i`: 전진, `k`: 정지, `,`: 후진
> - `j`: 좌회전, `l`: 우회전
> - `u`/`o`: 전진+회전
> - `q`/`z`: 속도 증가/감소

RViz에서 지도가 실시간으로 확장되는 것을 확인하세요.
환경의 모든 영역을 돌아다니면 완전한 지도가 만들어집니다.

> **매핑 팁**:
> - **천천히 이동** 권장 — 너무 빠르면 스캔 매칭이 실패하여 지도가 틀어집니다
> - RViz에서 지도가 이중으로 보이면 속도를 낮추세요 (`z`키로 감속)
> - 모든 벽/모서리가 선명하게 보이면 충분히 매핑된 것입니다

### 4.5 자동 매핑 (Auto Mapping)

텔레옵 대신 **explore_lite**를 사용하면 로봇이 미탐사 영역(frontier)을 자동으로 찾아 이동하며 지도를 만듭니다.

#### explore_lite 설치 (소스 빌드)

Jazzy apt 패키지가 없으므로 소스 빌드합니다:

```bash
# 워크스페이스 생성 및 소스 클론
mkdir -p ~/explore_ws/src
cd ~/explore_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git

# explore_lite와 메시지 패키지만 복사
cp -r m-explore-ros2/explore .
cp -r m-explore-ros2/explore_lite_msgs .
rm -rf m-explore-ros2

# 빌드
cd ~/explore_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

#### 자동 매핑 실행

터미널 1~4 (IsaacSim, swerve_controller, nav2_bridge, laser_merger)는 실행 중이어야 합니다.
텔레옵(4.4) 대신 아래 3개를 실행합니다:

**터미널 5 — SLAM Toolbox:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=$HOME/ms_AIworker/config/slam_params.yaml \
  use_sim_time:=true
```

**터미널 6 — Nav2 Navigation:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=$HOME/ms_AIworker/config/nav2_params.yaml
```

> `navigation_launch.py`만 사용합니다. SLAM Toolbox가 `map → odom` TF를 제공하므로 AMCL(`localization_launch.py`)은 필요 없습니다.

**터미널 7 — explore_lite:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
source ~/explore_ws/install/setup.bash

ros2 run explore_lite explore --ros-args \
  --params-file ~/ms_AIworker/config/explore_params.yaml
```

**터미널 8 — RViz (확인용):**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

rviz2 --ros-args -p use_sim_time:=true
```

RViz에서 `/map`, `/scan`, `/plan` 외에 frontier 시각화를 추가할 수 있습니다:
- `Add` → `By topic` → `/explore/frontiers` → `PointCloud2` — 모든 frontier (파란색)
- `Add` → `By topic` → `/explore/frontiers` 하위의 `MarkerArray` — 현재 목표 frontier (빨간색)

#### RViz에서 보이는 것

| 색상 | 의미 |
|------|------|
| **파란색** 포인트 | 감지된 모든 frontier — 미탐사/탐사 영역의 경계 |
| **빨간색** 마커 | 현재 선택된 목표 frontier — 로봇이 향하는 곳 |
| **초록색** 경로 | Nav2가 계획한 이동 경로 |

로봇이 빨간 frontier로 이동 → 도착 → 다음 frontier 선택 → 반복하며 지도가 자동으로 확장됩니다.
모든 frontier가 사라지면 탐색이 완료된 것입니다.

#### explore_params.yaml 주요 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `planner_frequency` | 0.33 | frontier 탐색 주기 (Hz) — 3초마다 |
| `progress_timeout` | 30.0 | 목표까지 진행 없으면 다음 frontier 선택 (초) |
| `min_frontier_size` | 0.75 | 최소 frontier 크기 (m) — 이보다 작은 frontier 무시 |
| `potential_scale` | 3.0 | 거리 비용 가중치 — 높을수록 가까운 frontier 선호 |
| `gain_scale` | 1.0 | frontier 크기 가중치 — 높을수록 큰 frontier 선호 |
| `return_to_init` | true | 탐색 완료 후 시작 위치로 복귀 |

> 매핑이 완료되면 explore_lite를 Ctrl+C로 종료하고, 아래 4.6에서 지도를 저장합니다.

### 4.6 지도 저장

매핑이 완료되면 지도를 저장합니다:

```bash
# 지도 저장 디렉토리 생성
mkdir -p ~/ms_AIworker/maps

# 지도 저장 (map.yaml + map.pgm 생성)
ros2 run nav2_map_server map_saver_cli -f ~/ms_AIworker/maps/map --ros-args -p use_sim_time:=true
```

지도 저장 성공 시 기대 출력:

```
[map_saver_cli]: Saving map to ~/ms_AIworker/maps/map
[map_saver_cli]: Map saved successfully
```

생성되는 파일:
- `map.yaml` — 지도 메타데이터 (해상도, 원점 등)
- `map.pgm` — 지도 이미지 (흰색=이동 가능, 검정=장애물, 회색=미탐사)

> 지도를 확인하려면: `eog ~/ms_AIworker/maps/map.pgm` (이미지 뷰어)

지도 저장 후 **SLAM Toolbox를 종료**합니다 (Ctrl+C).

---

## [5] 자율 주행 (Navigation)

저장된 지도를 사용해서 로봇이 목표 지점까지 자동으로 이동합니다.

### 5.1 Nav2 실행

터미널 1~4 (IsaacSim, swerve_controller, nav2_bridge, laser_merger)는 계속 실행 중입니다.
SLAM은 종료하고, 대신 Nav2를 실행합니다.

> **왜 두 개로 분리하나요?**
> `bringup_launch.py`는 `docking_server` 등 불필요한 노드를 포함해서 설정 오류가 발생할 수 있습니다.
> localization과 navigation을 분리 실행하면 필요한 노드만 올라옵니다.

**터미널 5A — Localization (AMCL) 실행:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true \
  map:=$HOME/ms_AIworker/maps/map.yaml \
  params_file:=$HOME/ms_AIworker/config/nav2_params.yaml
```

**터미널 5B — Navigation 실행:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=$HOME/ms_AIworker/config/nav2_params.yaml
```

> Nav2가 정상적으로 시작되면 각 터미널에서 `[lifecycle_manager]: Managed nodes are active` 메시지가 나옵니다.
> 이 메시지가 나올 때까지 기다리세요 (10~30초 소요).

**터미널 6 — RViz 실행:**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

rviz2 --ros-args -p use_sim_time:=true
```

> **주의**: `use_sim_time` 없이 RViz를 실행하면 TF 타이밍 에러가 발생합니다.

### 5.2 RViz에서 Nav2 설정

1. **Fixed Frame**: `map`

2. **디스플레이 추가** (`Add` 버튼):
   - `/map` → Map (Durability: Transient Local)
   - `/scan` → LaserScan (Reliability: Best Effort)
   - `/local_costmap/costmap` → Map (로컬 코스트맵)
   - `/global_costmap/costmap` → Map (글로벌 코스트맵)
   - `/plan` → Path (계획된 경로)

### 5.3 초기 위치 설정

AMCL은 로봇의 초기 위치를 알아야 합니다.

1. RViz 상단 메뉴에서 **`2D Pose Estimate`** 클릭
2. 지도 위에서 로봇의 **현재 위치를 클릭** → **방향으로 드래그**
3. AMCL 파티클이 수렴하면 위치 추정 완료

> **initial_pose는 IsaacSim에서 로봇의 시작 위치와 맞아야 합니다.**
> 로봇의 실제 시작 위치를 확인하려면:
> ```bash
> ros2 topic echo /odom --once --field pose.pose.position
> ```
> `nav2_params.yaml`의 `initial_pose`가 이 값과 다르면 지도에서 로봇 위치가 틀어집니다.
> 확인 후 `set_initial_pose: true`이면 자동으로 초기 위치가 설정됩니다.

### 5.4 목표 지점 설정 및 자율 이동

1. RViz 상단 툴바에서 **`2D Goal Pose`** 클릭
2. 지도 위에서 **목표 위치를 클릭** → **목표 방향으로 드래그**
3. Nav2가 자동으로:
   - 경로 계획 (초록색 선)
   - `/cmd_vel` 발행 → swerve_controller → 로봇 이동
   - 장애물 회피
   - 목표 도달 시 정지

```
[2D Goal Pose 클릭] (RViz2 상단 툴바)
     ↓
Nav2 Planner → 경로 계획 (/plan)
     ↓
Nav2 Controller → /cmd_vel 발행
     ↓
swerve_controller.py → 바퀴 명령
     ↓
IsaacSim → 로봇 이동
     ↓
swerve_controller.py → /joint_states 읽기 → /odom 업데이트 (FK)
     ↓
Nav2 → 경로 추종 반복...
```

---

## [6] 설정 파일 설명

### nav2_params.yaml 주요 파라미터

| 섹션 | 파라미터 | 값 | 설명 |
|------|---------|-----|------|
| controller | `desired_linear_vel` | 0.5 | 최대 직선 속도 (m/s) |
| controller | `lookahead_dist` | 0.4 | 경로 추종 전방 주시 거리 (m) |
| controller | `rotate_to_heading_angular_vel` | 2.0 | 제자리 회전 속도 (rad/s) |
| goal_checker | `xy_goal_tolerance` | 0.25 | 목표 도달 거리 허용 오차 (m) |
| goal_checker | `yaw_goal_tolerance` | 0.25 | 목표 방향 허용 오차 (rad) |
| local_costmap | `robot_radius` | 0.4 | 로봇 반지름 (m) — 충돌 판정용 |
| local_costmap | `width/height` | 4 | 로컬 코스트맵 크기 (m) |
| inflation | `inflation_radius` | 0.7 | 장애물 주변 안전 거리 (m) |
| planner | `tolerance` | 0.5 | 경로 계획 목표 허용 오차 (m) |
| velocity_smoother | `max_velocity` | [0.5, 0.0, 1.5] | 최대 [vx, vy, ω] |

### slam_params.yaml 주요 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `resolution` | 0.05 | 지도 해상도 (m/pixel) — 5cm |
| `max_laser_range` | 20.0 | LiDAR 최대 범위 (m) |
| `minimum_travel_distance` | 0.5 | 새 스캔 추가 최소 이동 거리 (m) |
| `minimum_travel_heading` | 0.5 | 새 스캔 추가 최소 회전 (rad) |
| `do_loop_closing` | true | 루프 클로징 활성화 |
| `map_update_interval` | 5.0 | 지도 업데이트 주기 (초) |

> 이 설정 파일들은 ROBOTIS ai_worker의 실제 로봇 설정을 기반으로 합니다.
> `use_sim_time: true`와 일부 파라미터만 IsaacSim에 맞게 수정되었습니다.

---

## [7] Troubleshooting

### /odom 토픽이 발행되지 않음

```bash
# TF 확인 — World → world → base_link 변환이 있는지
ros2 run tf2_tools view_frames

# 결과 PDF에서 World → world → base_link 경로가 있어야 함
# 없으면: IsaacSim Play + Step 4 TF publisher 확인
```

- IsaacSim이 **Play 상태**인지 확인
- Step 4의 TF Publisher Action Graph가 정상 동작하는지 확인
- `swerve_controller.py`가 실행 중인지 확인
- `/joint_states` 토픽이 발행되는지 확인: `ros2 topic hz /joint_states`
- Joint State Publisher가 IsaacSim에서 설정되었는지 확인 (섹션 3.0 참고)

### /scan 토픽이 없음

```bash
# 현재 발행 중인 LiDAR 토픽 확인
ros2 topic list | grep -i scan
ros2 topic list | grep -i laser

# laser_merger.py가 실행 중인지 확인
# /laser_scan_left 또는 /laser_scan_right가 있으면 laser_merger.py 실행
python3 ~/ms_AIworker/scripts/laser_merger.py
```

- Step 3에서 LiDAR를 설정했는지 확인
- IsaacSim이 Play 상태인지 확인

### SLAM에서 지도가 안 만들어짐

- `/scan` 데이터가 들어오는지 확인: `ros2 topic hz /scan`
  - 0Hz면 LiDAR가 동작하지 않는 것
- `/odom`이 업데이트되는지 확인: `ros2 topic hz /odom`
- TF tree가 완성되었는지 확인: `ros2 run tf2_tools view_frames`
  - `map → odom → World → world → base_link` 체인이 있어야 함
- 로봇이 실제로 **이동**해야 SLAM이 새 스캔을 추가합니다 (제자리에서는 지도가 안 늘어남)

### Nav2가 시작하자마자 에러

```
[lifecycle_manager]: Failed to change state ...
```

- TF timeout: `/odom`과 `/scan`이 발행되고 있는지 확인
- `use_sim_time: true`가 설정되었는지 확인 (모든 노드에)
- IsaacSim이 Play 상태인지 확인 (sim time이 진행되어야 함)

### 로봇이 목표까지 가다가 멈춤

- `progress_checker`의 `movement_time_allowance` 값 확인 (기본 10초)
  - 10초 동안 0.5m 이상 못 움직이면 실패 판정
- 속도가 너무 느리면 `desired_linear_vel` 증가
- 장애물에 막힌 경우: `inflation_radius` 감소

### RViz에서 지도/LiDAR가 안 보임

| 증상 | 해결 |
|------|------|
| 지도(Map)가 안 보임 | Durability Policy → **Transient Local** |
| LiDAR 점이 안 보임 | Reliability Policy → **Best Effort** |
| Fixed Frame 에러 | Fixed Frame → **map** (또는 **odom**) |
| 모든 것이 원점에 겹침 | TF tree 확인 — 빠진 변환이 있을 수 있음 |
| TF 타이밍 에러 | RViz 실행 시 `--ros-args -p use_sim_time:=true` 추가 |
| Depth 이미지가 세로 줄무늬/노이즈로 보임 | 아래 **Depth 이미지 표시 문제** 참고 |

### Depth 이미지 표시 문제 (RViz2)

RViz2에서 32FC1 depth 이미지가 **세로 줄무늬 노이즈**로 보이는 경우:

> **원인**: RViz2의 Image display가 32FC1 float depth를 자동 정규화할 때
> min/max 범위 계산이 실패하는 알려진 버그입니다 ([GitHub #512](https://github.com/ros2/rviz/issues/512)).
> `rqt_image_view`에서는 정상으로 보이는데 RViz에서만 깨지는 것이 특징입니다.

**해결:**

1. RViz에서 Image display 선택
2. **Normalize Range** → **체크 해제**
3. **Min Value** → `0.0`
4. **Max Value** → `10.0` (depth 카메라 max range에 맞춤)

**확인 방법** — 데이터 자체가 정상인지 확인:
```bash
# rqt_image_view로 확인 (이것이 정상이면 데이터는 문제 없음)
ros2 run rqt_image_view rqt_image_view /zed_mini/depth
```

### 실제 ROBOTIS와의 차이점

| 항목 | 실제 로봇 | IsaacSim |
|------|----------|----------|
| Odometry | swerve_drive_controller (ros2_control) | swerve_controller.py (FK 기반, 동일 방식) |
| LiDAR | dual_laser_merger → /scan | laser_merger.py → /scan |
| 제어 | ros2_control + hardware interface | swerve_controller.py → Action Graph |
| TF 발행 | robot_state_publisher + ros2_control | IsaacSim ROS2 Publish Transform Tree |
| 시간 | 실시간 (wall clock) | 시뮬레이션 시간 (use_sim_time: true) |

---

**Status**: VERIFIED — SLAM 매핑 및 Nav2 자율 주행 동작 확인
**이전**: [Step 6: Swerve Drive](06-swerve-drive.md)
**다음**: [Step 8: ApexNAV 개요](08-apexnav-overview.md)
