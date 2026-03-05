# IsaacSim 5.1.0 - AI Worker Setup Guide

NVIDIA IsaacSim 5.1.0 환경에서 **ROBOTIS FFW-SG2 Mobility AI Worker** 로봇을 시뮬레이션하기 위한 단계별 가이드입니다.

## Robot

| 항목 | 내용 |
|------|------|
| 모델 | ROBOTIS FFW-SG2 Mobility (AI Worker) |
| 타입 | Humanoid + Swerve Drive |
| 팔 | 7-DOF x 2 (양팔) + Gripper |
| 센서 | Zed X Mini, Intel D405 x 2, Ouster OS1-128, IMU, 2D LiDAR x 2 |
| URDF | [ROBOTIS-GIT/ai_worker](https://github.com/ROBOTIS-GIT/ai_worker) |

## Environment

| 항목 | 버전 |
|------|------|
| IsaacSim | 5.1.0 (pip install / Docker) |
| OS | Ubuntu 24.04 LTS |
| Python | 3.11 (conda) |
| ROS2 | Jazzy (IsaacSim internal bridge) |
| GPU | NVIDIA RTX PRO 6000 Blackwell |

## Guides

| Step | 내용 | 상태 |
|------|------|------|
| [01. Install IsaacSim](guides/01-install-isaacsim.md) | IsaacSim 5.1.0 설치, ROS2 Bridge 설정, conda 환경 구성 | Completed |
| [02. Import URDF](guides/02-import-urdf.md) | FFW-SG2 URDF 임포트, Stage 설정, Physics 구성 | Completed |
| [03. Import Sensors](guides/03-import-sensors.md) | 카메라, LiDAR, IMU 센서 추가 및 ROS2 연동 | Completed |
| [04. Publish TF Tree](guides/04-publish-tf.md) | ROS2 TF 트리 발행 (커스텀) | Completed |
| [09. Docker Setup](guides/09-docker-setup.md) | Docker로 IsaacSim 실행 (다른 PC에서 재현) | Completed |

## Quick Start

### 로컬 실행 (IsaacSim 설치된 PC)

```bash
# 1. conda 환경 활성화
conda activate isaac_sim

# 2. IsaacSim 실행
isaacsim

# 3. 저장된 World 열기
#    Content Browser → isaacsim_ai_worker/usd_ai_worker/Collected_World2/World2.usd

# 4. Play(▶) 후 별도 터미널에서 토픽 확인
conda activate isaac_sim
ros2 topic list
rviz2
```

### Docker 실행 (IsaacSim 미설치 PC)

NVIDIA Driver + Docker만 있으면 됩니다. 자세한 내용은 [Docker Setup 가이드](guides/09-docker-setup.md) 참고.

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

# 6. 호스트에서 토픽 확인 (ROS2 설치된 경우)
source /opt/ros/jazzy/setup.bash  # Humble이면 /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/fastdds.xml"
ros2 topic list
rviz2
```

## Sensors Configuration

```
FFW-SG2 Mobility AI Worker
├── head_link2
│   └── Zed X Mini (Stereo Camera + Depth + PCL)
├── arm_r_link7
│   └── d405 (Intel D405 - Depth Camera)
├── arm_l_link7
│   └── d405 (Intel D405 - Depth Camera)
├── base_link
│   ├── Ouster OS1-128 (3D LiDAR, 128ch, 1024 res)
│   ├── IMU (가속도, 각속도, 자세)
│   ├── SLAMTEC RPLIDAR S2E - Left  (2D LiDAR)
│   └── SLAMTEC RPLIDAR S2E - Right (2D LiDAR)
```

## ROS2 Topics

센서 데이터는 IsaacSim ROS2 Bridge를 통해 발행됩니다:

| Topic | Type | Sensor |
|-------|------|--------|
| `/zed_mini/depth/points` | PointCloud2 | Zed X Mini |
| `/zed_mini/depth_image` | Image | Zed X Mini (depth) |
| `/zed_mini/camera_info` | CameraInfo | Zed X Mini |
| `/imu/data` | Imu | IMU |
| `/laser_scan_left` | LaserScan | 2D LiDAR Left |
| `/laser_scan_right` | LaserScan | 2D LiDAR Right |
| `/point_cloud` | PointCloud2 | Ouster OS1-128 |
| `/tf` | TFMessage | TF Tree (전체 관절) |

## Project Structure

```
isaacsim-aiworker-guide/
├── Dockerfile                  # IsaacSim + ROS2 Jazzy CLI 커스텀 이미지
├── docker-compose.yml          # 컨테이너 실행 설정
├── fastdds.xml                 # FastDDS UDP 설정 (호스트↔컨테이너 통신)
├── guides/                     # 단계별 가이드
├── scripts/
│   └── docker-run.sh           # Docker 원클릭 실행
└── isaacsim_ai_worker/
    └── usd_ai_worker/
        └── Collected_World2/   # 센서 구성 완료된 World 파일
            └── World2.usd
```

## Save File

`isaacsim_ai_worker/usd_ai_worker/Collected_World2/` - 센서 및 TF 구성이 완료된 IsaacSim World 파일 (Collect As로 저장, 모든 에셋 포함)

## Notes

- 원본 문서는 IsaacSim 5.0.0 기준이며, 이 가이드는 **5.1.0** 기준으로 재작성됨
- 5.1.0의 주요 UI 차이: `New from Stage Template`, `Graph Editors > Action Graph`, `Collect As` 등
- ROS2 Bridge는 IsaacSim 내장 Humble libs 사용 (별도 ROS2 설치 불필요)
- Docker 이미지에는 ROS2 Jazzy CLI가 추가 설치되어 컨테이너 안에서 `ros2` 명령어 사용 가능
