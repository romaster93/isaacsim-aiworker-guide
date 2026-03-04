# Step 3: Import Sensors

## Overview
IsaacSim에서 FFW-SG2 로봇에 센서를 추가합니다.

| # | 센서 | 위치 | 방법 |
|---|------|------|------|
| 1-1 | Zed X Mini (카메라) | Head (head_link2) | Content 브라우저에서 드래그 |
| 1-2 | Intel D405 (카메라) | 양팔 (arm_r/l_link7) | Create > Camera (커스텀) |
| 2 | Ouster OS1-128 (3D LiDAR) | Base 상단 | Content 브라우저에서 드래그 |
| 3 | IMU | base_link | Create > Sensors > Imu Sensor |
| 4 | 2D LiDAR | base_link | Create > Sensors > PhysX LiDAR > Rotating |

## Prerequisites
- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] **URDF 임포트 (방법 A)** 완료 (Step 2) — 방법 B(USD 직접 열기)는 사용 불가!
- [x] Ground Plane 추가 완료

> **중요**: 반드시 **방법 A (URDF 임포트)**로 진행해야 합니다.
> 방법 B(USD 파일 직접 열기)는 ffw_sg2_follower가 Stage root가 되어
> 환경(Warehouse)이나 센서를 root 레벨에 추가할 수 없습니다.

### Stage 트리 구조 (방법 A 기준)
```
World (Stage root)
├── ffw_sg2_follower
│   └── world
│       └── base_link
│           └── ...
├── GroundPlane
└── full_warehouse  ← 환경
```

---

## [0] Environment 추가 (선택)

센서 추가 전에 시뮬레이션 환경을 추가할 수 있습니다.

### 절차

1. 하단 Content 브라우저에서 탐색:
   `Environments` → `Simple_Warehouse` → `full_warehouse.usd`
2. **Stage 패널의 빈 공간(root 레벨)**으로 드래그
3. Stage 트리에서 `full_warehouse`가 `ffw_sg2_follower`와 **같은 레벨(root)**에 있는지 확인

> **주의**: ffw_sg2_follower 트리 안으로 들어가면 안 됩니다.
> 반드시 Stage root 레벨에 배치하세요.

---

## [1-1] Zed X Mini 카메라 (Head)

AI Worker 헤드부분에 Zed Mini 카메라를 부착합니다.

> **참고**: Intel Realsense D405는 IsaacSim에서 직접 지원하지 않음.
> Zed X Mini만 Content 브라우저에서 바로 사용 가능.

### 절차

1. 하단 **Content 브라우저**에서 탐색:
   `Sensors` → `Stereolabs` → `ZED_X_Mini`
2. ZED_X_Mini를 **Stage의 ffw_sg2_follower 트리 안으로 드래그**
3. Stage에서 **head_link2** 하위로 이동
4. Top View, Front View로 전환하여 위치 정렬
5. Transform 설정:
   - Translate: **(0.03, 0.0, -0.025)**

### Physics 충돌 이슈 (5.0.0에서 발생)

원본 docs(5.0.0)에서는 헤드 내부 구조물과 ZED_X_MINI가 간섭하여 시뮬레이션 시 카메라가 튕겨나가는 문제 발생.
→ **Property > Physics > Rigidbody 삭제** 필요했음.
5.1.0에서는 발생하지 않을 수 있으므로, Play(▶)하여 확인 후 필요 시에만 삭제.

### 카메라 뷰 확인

- Viewport 상단 카메라 버튼 클릭 → `CameraLeft`, `CameraRight` 선택 가능
- 자물쇠 잠금 버튼으로 뷰 고정 권장 (실수로 카메라 뷰가 회전하는 것 방지)

---

## [1-2] Intel D405 카메라 (양팔)

AI Worker 양쪽 손목에 Intel Realsense D405를 부착합니다.
IsaacSim에서 직접 지원하지 않아 커스텀 카메라로 추가합니다.

### Instanceable 체크 해제 (필수)

arm_r_link7, arm_l_link7의 visuals에 Instanceable이 켜져 있으면 하위에 자식 prim(카메라 등)을 추가할 수 없음.
**5.1.0 + Create in Stage 모드에서도 Instanceable이 켜져 있는 경우가 있으므로 반드시 확인.**

1. `arm_r_link7` > `visuals` 클릭 → Property에서 **Instanceable 체크 해제**
2. `arm_l_link7` > `visuals` 클릭 → Property에서 **Instanceable 체크 해제**

### 오른팔 (CameraRightArm)

1. `Create` → `Camera`
2. 이름을 **CameraRightArm**으로 변경
3. Stage에서 **arm_r_link7 > visuals** 하위로 이동 (드래그)
4. Transform 설정:
   - Translate: **0, 0, 0**
   - Orient: **180.0, 0.0, 90.0**

### 왼팔 (CameraLeftArm)

1. `Create` → `Camera`
2. 이름을 **CameraLeftArm**으로 변경
3. Stage에서 **arm_l_link7 > visuals** 하위로 이동
4. Transform 설정:
   - Translate: **0, 0, 0**
   - Orient: **180.0, 0.0, 90.0**

### D405 카메라 Properties 설정 (양쪽 동일)

Intel D405 스펙 기반으로 계산한 값:
- 해상도: 1280 x 720, FOV: 87°(H) × 58°(V), 유효 깊이: 7cm~50cm

| 항목 | 값 | 설명 |
|------|------|------|
| **Focal Length** | **11.0** mm | 렌즈 초점 거리. FOV와 Aperture로부터 계산됨. 값이 작을수록 화각이 넓음 |
| **Focus Distance** | **400.0** | 초점이 맞는 거리 (렌더링용, 물리 시뮬에는 영향 없음) |
| **Horizontal Aperture** | **20.955** mm | 센서의 수평 크기. Focal Length와 함께 수평 FOV 결정 |
| **Vertical Aperture** | **12.21** mm | 센서의 수직 크기. Focal Length와 함께 수직 FOV 결정 |
| **Clipping Range** | X=**0.07**, Y=**0.5** | X=Near(최소 감지거리 7cm), Y=Far(최대 감지거리 50cm). 이 범위 밖 물체는 렌더링되지 않음 |

> **카메라 옵션 설명:**
> - **Focal Length**: 렌즈 초점 거리(mm). 작을수록 광각, 클수록 망원. D405는 87° 광각이라 11mm로 짧음
> - **Aperture (Horizontal/Vertical)**: 이미지 센서의 물리적 크기(mm). Focal Length와 조합하여 FOV를 결정함. FOV = 2 × atan(aperture / (2 × focal_length))
> - **Clipping Range**: 카메라가 볼 수 있는 최소/최대 거리. Near보다 가까운 물체, Far보다 먼 물체는 렌더링에서 제외됨
> - **Focus Distance**: Depth of Field(피사계 심도) 효과에 사용. 시뮬레이션 물리에는 무관
> - **Projection**: perspective(원근법) 또는 orthographic(직교). 일반 카메라는 perspective
> - **Stereo Role**: mono(단안) 또는 left/right(스테레오). 단일 카메라이므로 mono

---

## [1-3] Zed X Mini → ROS2 Depth + Point Cloud 발행

Action Graph를 사용하여 Zed X Mini 카메라의 depth 이미지와 point cloud를 ROS2 토픽으로 발행합니다.

### Action Graph 생성

`Window` → `Graph Editors` → `Action Graph` → **New Action Graph**

### 노드 추가 (우클릭 검색)

| 노드 | 역할 |
|------|------|
| **On Playback Tick** | 시뮬레이션 매 프레임마다 그래프를 실행하는 트리거 |
| **ROS2 Context** | ROS2 통신 컨텍스트(DDS 미들웨어 연결)를 제공 |
| **Isaac Create Render Product** | 카메라 prim으로부터 렌더링된 이미지(RGB, Depth 등) 생성 |
| **ROS2 Camera Helper** | 카메라 데이터(RGB/Depth/PCL)를 ROS2 토픽으로 발행 |
| **ROS2 Camera Info Helper** | 카메라 캘리브레이션 정보(intrinsics, distortion)를 camera_info 토픽으로 발행 |

### 노드 연결

```
On Playback Tick [Tick] ──→ Isaac Create Render Product [ExecIn]
On Playback Tick [Tick] ──→ ROS2 Camera Helper [ExecIn]
On Playback Tick [Tick] ──→ ROS2 Camera Info Helper [ExecIn]

ROS2 Context [Context] ──→ ROS2 Camera Helper [Context]
ROS2 Context [Context] ──→ ROS2 Camera Info Helper [Context]

Isaac Create Render Product [Render Product] ──→ ROS2 Camera Helper [Render Product]
Isaac Create Render Product [Render Product] ──→ ROS2 Camera Info Helper [Render Product]
```

### 속성 설정

**Isaac Create Render Product** (클릭 → Property):
- `cameraPrim`: Stage에서 Zed X Mini **CameraLeft** 선택 (Target 아이콘 클릭)

**ROS2 Camera Helper** (클릭 → Property):
- `type`: **depth_pcl** (depth 이미지 + point cloud 동시 발행)
- `topicName`: `/zed_mini/depth`
- `frameId`: `zed_mini_left`

> **type 옵션:**
> - `rgb`: RGB 이미지만
> - `depth`: depth 이미지만
> - `depth_pcl`: depth 이미지 + PointCloud2 동시 발행

**ROS2 Camera Info Helper** (클릭 → Property):
- `topicName`: `/zed_mini/camera_info`
- `topicNameRight`: (비워둠, 스테레오 Right용)
- `frameId`: `zed_mini_left`

### 발행되는 토픽

| 토픽 | 메시지 타입 | 내용 |
|------|------------|------|
| `/zed_mini/depth` | sensor_msgs/Image | Depth 이미지 |
| `/zed_mini/depth/points` | sensor_msgs/PointCloud2 | 3D Point Cloud |
| `/zed_mini/camera_info` | sensor_msgs/CameraInfo | 카메라 캘리브레이션 정보 |

### 테스트

1. **Play(▶)** 클릭
2. 별도 터미널에서:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
rviz2  # 시각화 확인
```

---

## [2] 3D LiDAR (Ouster OS1-128)

### 절차

1. Content 브라우저 → `Sensors` → `Ouster` → **OS1-128, 10Hz, 1024 res** 선택
2. Stage의 **ffw_sg2_follower > world > base_link** 하위로 드래그
3. Move로 Base 상단에 위치 조정

### ROS2 발행

`Tools` → `Robotics` → `ROS2 OmniGraphs` → `RTX Lidar` → LiDAR prim 선택

---

## [3] IMU (Inertial Measurement Unit)

### 절차

1. `Create` → `Sensors` → `Imu Sensor`
2. Stage에서 **ffw_sg2_follower > world > base_link** 하위로 이동
3. Transform 설정:
   - Translate: **(0.123, 0.0, 0.3595)**
   - Orient: **(0.0, 0.0, 0.0)**

### 알려진 이슈: velocity tensor size mismatch (5.1.0에서도 발생)

```
Incompatible size of velocity tensor in function getVelocities:
expected total size 6, received total size 12 with shape (2, 6)
```

**원인**: 같은 Articulation 안에 IMU가 2개 이상 있으면 발생.
Zed X Mini 카메라에 **내장 IMU**가 있어서 충돌함.

**해결**: Zed X Mini 트리 안의 IMU 센서 **삭제**. 우리가 추가한 IMU만 남김.

> **주의**: IsaacSim에서 같은 Articulation 내에 IMU 여러 개 사용 불가 (알려진 버그).
> 매니퓰레이터 등 여러 IMU가 필요한 경우 Python 스크립트 방식(`IMUSensor` 클래스)으로 우회 필요.

### Action Graph 설정 (ROS2 발행)

기존 카메라 Action Graph에 노드 추가:

| 노드 | 역할 |
|------|------|
| **Isaac Read IMU Node** | IMU 센서에서 가속도/각속도/자세 데이터 읽기 |
| **ROS2 Publish Imu** | IMU 데이터를 ROS2 토픽으로 발행 |
| **Isaac Read Simulation Time** | 시뮬레이션 타임스탬프 제공 (ExecIn 없음, 출력만 연결) |

**연결:**

| 출력 | → | 입력 |
|---|---|---|
| On Playback Tick → `tick` | → | Isaac Read IMU Node → `execIn` |
| Isaac Read IMU Node → `execOut` | → | ROS2 Publish Imu → `execIn` |
| Isaac Read IMU Node → `angVel` | → | ROS2 Publish Imu → `angularVelocity` |
| Isaac Read IMU Node → `linAccel` | → | ROS2 Publish Imu → `linearAcceleration` |
| Isaac Read IMU Node → `orientation` | → | ROS2 Publish Imu → `orientation` |
| Isaac Read Simulation Time → `simulationTime` | → | ROS2 Publish Imu → `timeStamp` |
| ROS2 Context → `context` | → | ROS2 Publish Imu → `context` |

> **주의**: ROS2 Publish Imu에는 `publishAngularVelocity`(bool)와 `angularVelocity`(double3)가 있음.
> 데이터 연결은 반드시 **`angularVelocity`(double3)**에. `publishAngularVelocity`는 ON/OFF 토글임.

**속성 설정:**
- Isaac Read IMU Node → `imuPrim`: Stage에서 IMU 센서 선택
- ROS2 Publish Imu → `topicName`: `/imu/data`, `frameId`: `base_link`

---

## [4] 2D LiDAR (Left / Right)

원본 docs는 PhysX LiDAR를 사용하지만, RTX LiDAR(SLAMTEC RPLIDAR S2E)를 사용해도 무방.

### 절차

1. Content 브라우저 → `Sensors` → `SLAMTEC` → `rplidar_s2e` 선택
2. Stage의 **ffw_sg2_follower > world > base_link** 하위로 드래그
3. **2개** 추가 (Left, Right)

| 이름 | Translate | Orient |
|------|-----------|--------|
| **LiDAR_left** | **(-0.112, 0.240, 0.343)** | **(0, 0, 0)** |
| **LiDAR_right** | **(-0.112, -0.240, 0.343)** | **(0, 0, 0)** |

### Property 설정
- **drawLines**: 체크
- **drawPoints**: 체크

### ROS2 발행

각각 `Tools` → `Robotics` → `ROS2 OmniGraphs` → `RTX Lidar` 로 연결

| LiDAR | frameId | topicName |
|-------|---------|-----------|
| Left | `lidar_2d_left` | `/laser_scan_left` |
| Right | `lidar_2d_right` | `/laser_scan_right` |

---

## Troubleshooting

### 센서가 시뮬레이션 시 튕겨나감
- 센서의 Physics > Rigidbody 삭제
- 센서가 로봇 메시와 겹치지 않도록 위치 조정

### ROS2 Topic 발행 실패
- `isaacsim.ros2.bridge` 확장이 CUDA/torch 에러로 로드 실패할 수 있음 (Step 1 참고)
- 이 경우 ROS2 연동은 CUDA 라이브러리 문제 해결 후 진행

### IMU angular_velocity 에러
- IMU prim 위치를 변경하여 테스트
- 5.1.0에서 해결되었을 수 있음

### 카메라 이미지가 검은색
- Viewport에서 조명(Stage Lights) 활성화 확인
- 카메라 방향이 올바른지 확인 (Orient 값 재확인)

---
**Status**: COMPLETED
**완료된 센서**: [0] Environment, [1-1] Zed X Mini, [1-2] D405 양팔, [1-3] ROS2 Depth+PCL, [2] 3D LiDAR, [3] IMU, [4] 2D LiDAR Left/Right
**저장된 파일**: `/home/cho/ms_AIworker/isaacsim_ai_worker/usd_ai_worker/ffw_sg2_follower_working/`
**실행 명령** (ROS2 포함):
```bash
conda activate isaac_sim
unset LD_PRELOAD
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="/home/cho/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/exts/3/isaacsim.ros2.bridge-4.12.4+107.3.3.lx64/jazzy/lib"
isaacsim
```
**Next**: [Step 4: Control Humanoids](04-control-humanoids.md)
