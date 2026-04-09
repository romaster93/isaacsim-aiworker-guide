# IsaacSim 5.1.0 AI Worker - 자율 탐색 가이드

NVIDIA IsaacSim 5.1.0 + ROS2 Jazzy 환경에서 **ROBOTIS FFW-SG2 Swerve 로봇**을 시뮬레이션하고, **ApexNAV** 기반 자율 탐색을 구현하는 단계별 가이드입니다.

## 로봇 사양

| 항목 | 내용 |
|------|------|
| 모델 | ROBOTIS FFW-SG2 Mobility (AI Worker) |
| 타입 | Humanoid + Swerve Drive |
| 팔 | 7-DOF x 2 (양팔) + Gripper |
| 센서 | Zed X Mini, Intel D405 x 2, Ouster OS1-128, IMU, 2D LiDAR x 2 |
| URDF | [ROBOTIS-GIT/ai_worker](https://github.com/ROBOTIS-GIT/ai_worker) |

## 개발 환경

| 항목 | 버전 |
|------|------|
| IsaacSim | 5.1.0 |
| OS | Ubuntu 24.04 LTS |
| ROS2 | Jazzy |
| GPU | NVIDIA RTX PRO 6000 Blackwell |
| ApexNAV | [romaster93/ApexNav_ROS2_wrapper](https://github.com/romaster93/ApexNav_ROS2_wrapper) |

## 가이드 목록

| 번호 | 파일 | 내용 | 상태 |
|------|------|------|------|
| 00 | [00-docker-setup.md](guides/00-docker-setup.md) | Docker로 IsaacSim 실행 (다른 PC 재현용) | 완료 |
| 01 | [01-install-isaacsim.md](guides/01-install-isaacsim.md) | IsaacSim 5.1.0 설치, ROS2 Bridge, conda 환경 구성 | 완료 |
| 02 | [02-import-urdf.md](guides/02-import-urdf.md) | FFW-SG2 URDF 임포트, Stage 설정, Physics 구성 | 완료 |
| 03 | [03-import-sensors.md](guides/03-import-sensors.md) | 카메라, LiDAR, IMU 센서 추가 및 ROS2 연동 | 완료 |
| 04 | [04-publish-tf.md](guides/04-publish-tf.md) | ROS2 TF 트리 발행 (커스텀) | 완료 |
| 05 | [05-control-humanoids.md](guides/05-control-humanoids.md) | ROS2 JointState로 관절 제어 | 완료 |
| 06 | [06-swerve-drive.md](guides/06-swerve-drive.md) | /cmd_vel → Swerve IK → 바퀴 제어 | 완료 |
| 07 | [07-navigation-system.md](guides/07-navigation-system.md) | Nav2 자율 주행 (SLAM + AMCL + FK Odometry) | 완료 |
| 08 | [08-apexnav-overview.md](guides/08-apexnav-overview.md) | ApexNAV 개요 (Semantic Frontier Exploration) | 완료 |
| 09 | [09-apexnav-bridge.md](guides/09-apexnav-bridge.md) | IsaacSim ↔ ApexNAV 토픽 브릿지 구성 | 완료 |
| 10 | [10-apexnav-autonomous.md](guides/10-apexnav-autonomous.md) | ApexNAV 자율 탐색 실행 (Phase 2) | 완료 |
| 11 | [11-apexnav-vlm.md](guides/11-apexnav-vlm.md) | VLM 기반 물체 탐색 (Phase 3 — 미완성) | 진행 중 |
| 12 | [12-interioragent-dataset.md](guides/12-interioragent-dataset.md) | InteriorAgent 실내 씬 데이터셋 활용 | 완료 |

## 빠른 시작

### Docker 실행 (권장)

NVIDIA Driver + Docker만 있으면 됩니다. 자세한 내용은 [00-docker-setup.md](guides/00-docker-setup.md) 참고.

```bash
# 1. 클론
git clone https://github.com/romaster93/isaacsim-aiworker-guide.git
cd isaacsim-aiworker-guide

# 2. NGC 로그인 & 베이스 이미지 Pull
docker login nvcr.io    # Username: $oauthtoken, Password: NGC API Key
docker pull nvcr.io/nvidia/isaac-sim:5.1.0

# 3. 커스텀 이미지 빌드 (ROS2 Jazzy CLI + FastDDS 설정 포함)
docker compose build

# 4. 실행
chmod +x scripts/docker-run.sh
./scripts/docker-run.sh gui
# 컨테이너 안에서: ./runapp.sh

# 5. World 열기
#    Content Browser → /isaac-sim/workspace/usd_ai_worker/Collected_World2/World2.usd

# 6. 호스트에서 토픽 확인
source scripts/ros2-bridge-env.sh
ros2 topic list
```

### 로컬 실행 (IsaacSim 설치된 PC)

```bash
# 1. IsaacSim 실행
conda deactivate   # conda base와 ROS2 Jazzy Python 충돌 방지
isaacsim

# 2. World 열기
#    Content Browser → isaacsim_ai_worker/usd_ai_worker/Collected_World2/World2.usd

# 3. Play(▶) 후 별도 터미널에서 스크립트 실행
source scripts/ros2-bridge-env.sh
ros2 run ms_aiworker swerve_controller.py
```

## 주요 스크립트

| 스크립트 | 역할 |
|----------|------|
| `scripts/swerve_controller.py` | `/cmd_vel` → Swerve IK → joint commands 발행 + joint_states → `/odom` FK |
| `scripts/nav2_bridge.py` | odom → `World` static TF 발행 (Nav2 연동) |
| `scripts/isaacsim_apexnav_bridge.py` | IsaacSim ↔ ApexNAV 토픽 변환 (depth 정규화, 좌표 변환) |
| `scripts/swerve_path_follower.py` | trajectory 추종 (feedforward + P 제어) |
| `scripts/target_label_publisher.py` | 탐색 목표 물체 CLI 발행 (초기 360° 회전 포함) |
| `scripts/laser_merger.py` | 양쪽 2D LiDAR 병합 → `/scan` |
| `scripts/nav2_bridge.py` | odom → World static TF (Nav2 연동) |

## 주요 설정 파일

| 파일 | 역할 |
|------|------|
| `config/apexnav_bridge.yaml` | depth 카메라 파라미터 공유 설정 (fx/fy=245.33, 정규화 등) |
| `config/nav2_params.yaml` | Nav2 파라미터 (ROBOTIS 기반, use_sim_time: true) |
| `config/slam_params.yaml` | SLAM Toolbox 설정 |
| `fastdds.xml` | FastDDS UDP 설정 (호스트↔Docker 통신) |

## 센서 구성

```
FFW-SG2 Mobility AI Worker
├── head_link2
│   └── Zed X Mini (RGB + Depth + PointCloud, 640x480)
├── arm_r_link7
│   └── Intel D405 (Depth Camera)
├── arm_l_link7
│   └── Intel D405 (Depth Camera)
└── base_link
    ├── Ouster OS1-128 (3D LiDAR, 128ch)
    ├── IMU
    ├── SLAMTEC RPLIDAR S2E - Left  (2D LiDAR)
    └── SLAMTEC RPLIDAR S2E - Right (2D LiDAR)
```

## ROS2 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/zed_mini/rgb` | Image | Zed X Mini RGB |
| `/zed_mini/depth` | Image | Zed X Mini Depth (32FC1, 미터) |
| `/zed_mini/camera_info` | CameraInfo | 카메라 내부 파라미터 |
| `/zed_mini/depth/points` | PointCloud2 | Zed X Mini 포인트 클라우드 |
| `/point_cloud` | PointCloud2 | Ouster OS1-128 |
| `/imu/data` | Imu | IMU |
| `/laser_scan_left`, `/laser_scan_right` | LaserScan | 2D LiDAR 좌/우 |
| `/scan` | LaserScan | 병합된 2D LiDAR |
| `/tf` | TFMessage | TF 트리 전체 |
| `/joint_states` | JointState | 관절 상태 |
| `/odom` | Odometry | FK 기반 odometry |
| `/cmd_vel` | Twist | 이동 명령 |
| `/isaac_sim/joint_commands` | JointState | 관절 제어 명령 |

## 프로젝트 구조

```
ms_AIworker/
├── Dockerfile                      # IsaacSim + ROS2 Jazzy CLI 커스텀 이미지
├── docker-compose.yml              # 컨테이너 실행 설정
├── fastdds.xml                     # FastDDS UDP 설정
├── guides/                         # 단계별 가이드 (00~12)
├── config/
│   ├── apexnav_bridge.yaml         # ApexNAV 브릿지 depth 파라미터
│   ├── nav2_params.yaml            # Nav2 설정
│   └── slam_params.yaml            # SLAM Toolbox 설정
├── scripts/
│   ├── swerve_controller.py        # Swerve IK + FK odometry
│   ├── nav2_bridge.py              # odom → World static TF
│   ├── isaacsim_apexnav_bridge.py  # IsaacSim ↔ ApexNAV 브릿지
│   ├── swerve_path_follower.py     # trajectory 추종
│   ├── target_label_publisher.py   # 탐색 목표 물체 발행
│   ├── laser_merger.py             # 2D LiDAR 병합
│   ├── docker-run.sh               # Docker 원클릭 실행
│   └── ros2-bridge-env.sh          # 호스트 ROS2 환경 설정
└── isaacsim_ai_worker/
    └── usd_ai_worker/
        └── Collected_World2/       # 센서 구성 완료된 World 파일
            └── World2.usd
```

## 현재 개발 상태 (2026-04-09 기준)

| 단계 | 내용 | 상태 |
|------|------|------|
| Phase 1 | IsaacSim 환경 구성 (URDF, 센서, TF, 제어) | 완료 |
| Phase 2 | ApexNAV 자율 탐색 (SDF 맵 생성 + Frontier 탐색) | 완료 |
| Phase 3 | VLM 기반 물체 탐색 (target_label → 시맨틱 탐색) | 진행 중 |

## 주요 설계 사항

- **TF 기준 프레임**: `World` (대문자, stage root, 고정). `world` (소문자)는 로봇과 함께 이동하므로 사용 금지
- **depth 정규화**: meters / 5.0 → [0, 1] (ApexNAV C++ planner 입력 형식)
- **카메라 파라미터**: fx=fy=245.33 (camera_info K 행렬 실측값)
- **FastDDS**: Docker 경계를 넘을 때 Shared Memory 불가 → UDP 강제 (`ros2-bridge-env.sh`)
- **conda 주의**: base Python 3.13과 ROS2 Jazzy Python 3.12 충돌 → 실행 전 `conda deactivate` 필요

## 참고

- IsaacSim 5.1.0은 5.0.0과 UI가 다름 (`New from Stage Template`, `Graph Editors > Action Graph`, `Collect and Save As` 등)
- ApexNAV는 밀폐 실내 환경 전제 설계 — 개방 환경에서는 C++ 파라미터 튜닝 필요
- 가이드에서 발견한 문제나 차이점은 즉시 해당 md 파일에 반영
