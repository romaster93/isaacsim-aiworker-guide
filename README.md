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
| IsaacSim | 5.1.0 (pip install) |
| OS | Ubuntu 24.04 LTS |
| Python | 3.11 (conda) |
| ROS2 | Jazzy (IsaacSim internal bridge) |
| GPU | NVIDIA RTX PRO 6000 Blackwell |

## Guides

| Step | 내용 | 상태 |
|------|------|------|
| [01. Install IsaacSim](guides/01-install-isaacsim.md) | IsaacSim 5.1.0 설치, ROS2 Bridge 설정, conda 환경 구성 | **Completed** |
| [02. Import URDF](guides/02-import-urdf.md) | FFW-SG2 URDF 임포트, Stage 설정, Physics 구성 | **Completed** |
| [03. Import Sensors](guides/03-import-sensors.md) | 카메라, LiDAR, IMU 센서 추가 및 ROS2 연동 | **Completed** |
| [04. Publish TF Tree](guides/04-publish-tf.md) | ROS2 TF 트리 발행 (커스텀) | **In Progress** |
| [05. Control Humanoids](guides/05-control-humanoids.md) | 휴머노이드 제어 | Pending |
| [06. Swerve Drive](guides/06-swerve-drive.md) | Swerve Drive 제어 | Pending |
| [07. Kinematic Override](guides/07-kinematic-override.md) | Kinematic Override Drive | Pending |
| [08. Navigation System](guides/08-navigation-system.md) | Navigation 시스템 | Pending |

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

## Quick Start

```bash
# 1. conda 환경 활성화 (ROS2 env vars 자동 설정됨)
conda activate isaac_sim

# 2. IsaacSim 실행
isaacsim

# 3. 저장된 World 열기
#    Content Browser → isaacsim_ai_worker/usd_ai_worker/Collected_World2/World2.usd

# 4. Play(▶) 후 별도 터미널에서 토픽 확인
source /opt/ros/jazzy/setup.bash
ros2 topic list
rviz2
```

## Save File

`isaacsim_ai_worker/usd_ai_worker/Collected_World2/` - 센서 구성이 완료된 IsaacSim World 파일 (Collect As로 저장)

## Notes

- 원본 문서는 IsaacSim 5.0.0 기준이며, 이 가이드는 **5.1.0** 기준으로 재작성됨
- 5.1.0의 주요 UI 차이: `New from Stage Template`, `Graph Editors > Action Graph`, `Collect As` 등
- ROS2 Bridge는 IsaacSim 내장 Jazzy libs 사용 (별도 ROS2 설치 불필요)
