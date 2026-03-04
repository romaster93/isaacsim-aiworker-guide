# Step 4: ROS2 TF Tree 발행

## Overview
로봇의 모든 링크 및 센서 좌표계를 ROS2 TF로 발행합니다.
TF가 없으면 RViz2에서 센서 데이터의 위치를 알 수 없습니다 ("No tf data" 에러 발생).

> **참고**: 이 가이드는 원본 docs에 없는 **커스텀 가이드**입니다.
> IsaacSim 5.1.0 공식 문서를 기반으로 작성되었습니다.

### TF란?
- ROS2에서 각 좌표계(frame) 간의 상대적 위치/회전 관계를 나타내는 트리 구조
- 예: `base_link` → `head_link2` → `CameraLeft` 의 변환 체인
- `/tf` (동적 변환: 조인트), `/tf_static` (정적 변환: 고정 센서) 두 토픽으로 발행
- TF가 있어야 RViz2에서 센서 데이터를 올바른 위치에 시각화 가능

## Prerequisites
- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 완료 (Step 2)
- [x] 센서 추가 완료 (Step 3)
- [x] ROS2 Bridge 동작 확인

---

## [1] TF Publisher 생성

### 방법 1: 메뉴 자동 생성 (권장)

1. `Tools` → `Robotics` → `ROS2 OmniGraphs` → **TF Publisher**
2. Target Prim 선택 창이 나옴 → **ffw_sg2_follower** 선택
3. **OK** 클릭

> **경고 메시지가 뜨는 경우:**
> ```
> Warning: Ill-formed SdfPath <>
> Could not sync USD on attribute inputs:parentPrim
> ```
> 이것은 **정상**입니다. `parentPrim`이 비어있어서 나오는 경고이며 에러가 아닙니다.
> parentPrim을 비워두면 World가 기본 부모 frame이 되는데, 빈 경로를 USD에 쓰려다 경고가 뜨는 것입니다.
> TF 발행 자체에는 영향 없습니다.

#### 자동 생성되는 Action Graph 노드

| 노드 | 역할 |
|------|------|
| **On Playback Tick** | 매 프레임 실행 트리거 |
| **Isaac Read Simulation Time** | 시뮬레이션 타임스탬프 제공 |
| **ROS2 Publish Transform Tree** | 로봇의 전체 TF 트리를 /tf, /tf_static으로 발행 |

### 방법 2: Action Graph 수동 구성

기존 센서 Action Graph에 TF 노드를 직접 추가할 수도 있습니다.

1. 기존 Action Graph 열기: `Window` → `Graph Editors` → `Action Graph`
2. 우클릭 → 노드 검색 → **ROS2 Publish Transform Tree** 추가

**노드 연결:**
```
On Playback Tick [Tick] ──→ ROS2 Publish Transform Tree [ExecIn]
Isaac Read Simulation Time [simulationTime] ──→ ROS2 Publish Transform Tree [timeStamp]
ROS2 Context [Context] ──→ ROS2 Publish Transform Tree [Context]
```

> **참고**: Isaac Read Simulation Time은 ExecIn이 없습니다. 출력만 연결하면 됩니다.

---

## [2] targetPrims 설정 (중요)

TF Publisher를 생성한 직후에는 **URDF 링크만** TF에 나옵니다.
수동 추가한 센서(Zed X Mini, LiDAR 등)는 TF에 포함되지 않습니다.

### Articulation Root 확인

> **주의**: `ffw_sg2_follower`를 targetPrim으로 넣으면 **ffw 한 개만 TF에 나옴**
> 하위 링크(base_link, head_link 등)가 나오지 않습니다.
>
> URDF 임포트 시 Articulation Root는 `ffw_sg2_follower` 자체가 아니라
> **ffw_sg2_follower > world** 에 있습니다.

**Articulation Root 확인 방법:**
1. Stage에서 `ffw_sg2_follower > world` 클릭
2. Property에 **Articulation Root** 항목이 있으면 이것이 실제 Articulation Root

### targetPrims 수정

Action Graph → **ROS2 Publish Transform Tree** 노드 클릭 → Property:

| 항목 | 값 | 설명 |
|------|------|------|
| **targetPrims** | `ffw_sg2_follower > world` | Articulation Root가 있는 prim. 하위 **모든 URDF 링크**가 자동 포함됨 |
| **parentPrim** | `World` (Stage root) | 고정 기준 좌표계. IsaacSim Stage의 최상위 World를 선택 |

> **Articulation Root란?**
> IsaacSim에서 관절로 연결된 로봇의 **물리 시뮬레이션 루트**입니다.
> - 이 prim 아래의 모든 링크/조인트가 하나의 물리 단위(articulation)로 묶임
> - 조인트 제어(위치/속도/힘), 역기구학 등이 이 루트 기준으로 동작
> - URDF 임포트 시 자동으로 설정됨

### parentPrim을 World로 설정하는 이유

Moveable Base로 임포트했기 때문에 URDF의 `world` 링크도 **로봇과 함께 움직입니다**.
parentPrim을 비워두면 고정 기준 좌표계가 없어서, RViz2에서 모든 frame이 같이 움직입니다.

**parentPrim을 Stage의 `World`로 설정하면:**
```
World (고정 - IsaacSim Stage root)
└── world (로봇 루트, 로봇과 함께 이동)
    └── base_link
        └── ...
```
`World`가 고정 좌표계 역할을 하여 로봇의 절대 위치를 알 수 있습니다.

---

## [3] 센서 prim을 TF에 추가

URDF에 정의된 링크는 자동으로 TF에 포함되지만, **수동 추가한 센서 prim은 포함되지 않습니다**.
센서 데이터를 RViz2에서 올바르게 시각화하려면 센서 prim도 targetPrims에 추가해야 합니다.

### 카메라 Optical Frame 문제 (중요)

카메라 센서를 TF에 추가할 때 **좌표축 컨벤션**을 이해해야 합니다.

**로봇과 카메라는 좌표축이 다릅니다:**

```
로봇 좌표계 (REP-103)          카메라 Optical Frame

      Z (위)                        Y (아래)
      |                             |
      |                             |
      +------ Y (좌)               +------ X (우)
     /                             /
    X (전방)                      Z (전방 = 촬영 방향)
```

| 축 | 로봇 frame | 카메라 optical frame |
|----|-----------|-------------------|
| **X** | 전방 (로봇 진행 방향) | 우측 (이미지 가로) |
| **Y** | 좌측 | 하방 (이미지 세로) |
| **Z** | 상방 | 전방 (촬영 방향) |

> **왜 다른가?**
> 이것은 IsaacSim만의 문제가 아니라 **컴퓨터 비전의 표준 컨벤션**입니다.
> 카메라 이미지의 원점은 좌상단이고, 가로=X, 세로=Y, 깊이=Z로 정의합니다.
> 반면 로봇은 ROS REP-103 표준을 따릅니다 (X=전방, Y=좌, Z=위).
>
> ROS2에서는 보통 두 개의 frame을 사용합니다:
> - `camera_link` — 카메라의 물리적 위치 (로봇 좌표계)
> - `camera_optical_frame` — 카메라의 촬영 좌표계 (optical 컨벤션)
>
> 이 둘 사이에 회전 변환(static transform)이 존재합니다.

### 해결: 카메라 prim을 targetPrims에 추가

IsaacSim의 카메라 prim(CameraLeft 등)은 이미 **optical frame 회전을 포함**하고 있습니다.
따라서 카메라 prim을 targetPrims에 추가하면 올바른 축 변환이 TF에 자동 반영됩니다.

**frameId를 `head_link2`로 설정하면 안 되는 이유:**
- depth/PCL 데이터는 **optical frame 기준**으로 발행됨
- `head_link2`는 로봇 좌표계 (X=전방, Z=위)
- optical 데이터를 로봇 좌표계에 표시하면 **축이 90° 회전되어 데이터가 엉뚱한 방향으로 보임**

### 센서 targetPrims 추가 절차

ROS2 Publish Transform Tree 노드 → Property → **targetPrims**에서 **+ Add Target**으로 추가:

| 센서 | 추가할 prim 경로 | 설명 |
|------|-----------------|------|
| Zed X Mini | `head_link2 > ZED_X_Mini > base_link > ZED_X_Mini > CameraLeft` | optical frame 포함 |

> **참고**: Zed X Mini의 Stage 트리 구조는 중첩되어 있습니다:
> `head_link2 > ZED_X_Mini > base_link > ZED_X_Mini > CameraLeft`
> 가장 하위의 **CameraLeft**를 선택해야 합니다.

### 센서 frameId 수정

targetPrims에 센서를 추가한 후, 각 센서의 ROS2 발행 노드에서 **frameId를 실제 TF frame 이름과 일치**시켜야 합니다.

| 센서 | ROS2 노드 | 기존 frameId | 변경 frameId |
|------|----------|-------------|-------------|
| Zed X Mini (depth_pcl) | ROS2 Camera Helper | `zed_mini_left` | **`CameraLeft`** |
| Zed X Mini (depth) | ROS2 Camera Helper | `zed_mini_left` | **`CameraLeft`** |
| Zed X Mini (camera_info) | ROS2 Camera Info Helper | `zed_mini_left` | **`CameraLeft`** |
| IMU | ROS2 Publish Imu | `base_link` | `base_link` (변경 없음) |
| 2D LiDAR Left | (RTX Lidar 자동 생성) | `lidar_2d_left` | **`base_link`** |
| 2D LiDAR Right | (RTX Lidar 자동 생성) | `lidar_2d_right` | **`base_link`** |

> **frameId 규칙**: 센서의 frameId는 **TF 트리에 실제 존재하는 frame 이름**과 정확히 일치해야 합니다.
> 일치하지 않으면 RViz2에서 해당 센서 데이터를 표시할 수 없습니다.
> `ros2 run tf2_tools view_frames`로 실제 frame 이름을 확인하세요.

---

## [4] 테스트

### TF 토픽 확인

```bash
source /opt/ros/jazzy/setup.bash

# TF 토픽 확인
ros2 topic list | grep tf

# TF 데이터 확인 (한 번만)
ros2 topic echo /tf --once
```

정상이면 아래처럼 출력:
```
transforms:
- header:
    stamp: {sec: ..., nanosec: ...}
    frame_id: world
  child_frame_id: base_link
  transform:
    translation: {x: ..., y: ..., z: ...}
    rotation: {x: ..., y: ..., z: ..., w: ...}
```

### TF 트리 시각화 (PDF)

```bash
ros2 run tf2_tools view_frames
# frames_YYYY-MM-DD_HH.MM.SS.pdf 파일이 홈 디렉토리에 생성됨
evince ~/frames_*.pdf
```

정상적인 TF 트리:
```
World (고정)
└── world
    └── base_link
        ├── head_link1 → head_link2 → ZED_X_Mini → ... → CameraLeft
        ├── lift_link
        ├── arm_base_link
        ├── arm_r_link1~7 → camera_r_link, gripper_r_...
        ├── arm_l_link1~7 → camera_l_link, gripper_l_...
        ├── left_wheel_steer_link → left_wheel_drive_link
        ├── right_wheel_steer_link → right_wheel_drive_link
        ├── rear_wheel_steer_link → rear_wheel_drive_link
        └── sensor_lidar_imu_link
```

### RViz2에서 확인

```bash
rviz2
```

1. **Add** → **By display type** → **TF** 선택 → OK
2. **Global Options** → **Fixed Frame**: `World` (고정 프레임)
3. Viewport에 좌표축(빨강=X, 초록=Y, 파랑=Z)이 각 링크 위치에 표시되면 정상
4. 센서 데이터(PointCloud2, Image 등)도 올바른 위치/방향에 표시되는지 확인

---

## IsaacSim TF Viewer (선택)

IsaacSim 내부에서도 TF를 시각화할 수 있습니다:

1. `Window` → **TF Viewer** (isaacsim.ros2.tf_viewer 확장이 활성화되어 있어야 함)
2. Root frame 선택 (예: `World`)
3. Viewport에 frame 마커, 축, 연결선이 표시됨

> **5.1.0 주의**: TF Viewer 확장이 기본 비활성화일 수 있습니다.
> `Window` → `Extensions` → `isaacsim.ros2.tf_viewer` 검색 → 활성화

---

## Troubleshooting

### ffw_sg2_follower를 targetPrim으로 넣었는데 하위 링크가 TF에 안 나옴
- `ffw_sg2_follower`는 Articulation Root가 아닙니다
- **`ffw_sg2_follower > world`**가 실제 Articulation Root
- targetPrims를 `ffw_sg2_follower > world`로 변경하세요

### 카메라 depth/PCL 데이터가 엉뚱한 방향으로 표시됨
- **카메라 optical frame 축 문제**입니다 (위 [3] 섹션 참고)
- frameId를 `head_link2`(로봇 축)가 아닌 **`CameraLeft`(optical 축)**로 설정
- CameraLeft prim이 targetPrims에 추가되어 TF에 존재해야 함

### RViz2에서 "No tf data" 에러
- TF Publisher가 **Play 중에만** 발행합니다. 시뮬레이션이 실행 중인지 확인
- Fixed Frame을 TF 트리에 존재하는 frame으로 변경
  - `map`은 없음 → `World` 또는 `base_link`로 변경
- `ros2 topic echo /tf`로 데이터가 오는지 확인

### world와 base_link가 같이 움직임 (고정 좌표계 없음)
- Moveable Base로 임포트했기 때문에 URDF의 `world` 링크도 로봇과 함께 이동
- **parentPrim을 Stage의 `World`(최상위)로 설정**하면 고정 좌표계가 추가됨
- RViz2 Fixed Frame을 `World`로 설정

### 센서 frame이 TF 트리에 없음
- URDF 정의 링크만 자동 포함됨. 수동 추가 센서는 포함 안 됨
- ROS2 Publish Transform Tree → **targetPrims** → **+ Add Target**으로 센서 prim 수동 추가

### TF 트리가 끊겨있음 (view_frames에서 별도 트리)
- parentPrim 설정 확인
- Articulation Root가 올바른 링크(`ffw_sg2_follower > world`)인지 확인

### /tf_static이 발행되지 않음
- 고정 조인트(fixed joint)의 변환은 /tf_static으로 발행됨
- ROS2 Publish Transform Tree 노드 연결 확인

---
**Status**: COMPLETED
**Next**: [Step 5: Control Humanoids](05-control-humanoids.md)
