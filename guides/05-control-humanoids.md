# Step 4: Control Humanoids

## Overview
ROS2를 통해 FFW-SG2 로봇의 관절을 제어합니다.
- IsaacSim Action Graph에서 ROS2 Joint State 구독
- Python에서 관절 명령 발행
- 정현파 테스트 코드로 모든 관절 검증

## Prerequisites
- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 완료 (Step 2)
- [x] 센서 구성 완료 (Step 3)
- [x] ROS 2 Jazzy 설치 완료

## Step 4.1: 로봇 관절 구성

FFW-SG2의 관절 목록과 범위:

| 관절 그룹 | 관절명 | DOF 수 | 범위 | 설명 |
|---------|--------|--------|------|------|
| **팔** | arm_l_joint1~7 | 7 | ±180° | 왼쪽 팔 (7-DOF) |
| | arm_r_joint1~7 | 7 | ±180° | 오른쪽 팔 (7-DOF) |
| **머리** | head_joint1 | 1 | ±45° | 머리 피치 (위아래) |
| | head_joint2 | 1 | ±90° | 머리 요 (좌우) |
| **그리퍼** | gripper_l_joint1~4 | 4 | 0~1 (정규화) | 왼쪽 그리퍼 (입력 1개, 운동학으로 계산) |
| | gripper_r_joint1~4 | 4 | 0~1 (정규화) | 오른쪽 그리퍼 (입력 1개, 운동학으로 계산) |
| **리프트** | lift_joint | 1 | 0~500 mm | 상승/하강 |
| **휠** | left_wheel_steer | 1 | -90~90° | 왼쪽 휠 스티어 |
| | left_wheel_drive | 1 | -360~360°/s | 왼쪽 휠 구동 |
| | right_wheel_steer | 1 | -90~90° | 오른쪽 휠 스티어 |
| | right_wheel_drive | 1 | -360~360°/s | 오른쪽 휠 구동 |
| | rear_wheel_steer | 1 | -90~90° | 뒷 휠 스티어 |
| | rear_wheel_drive | 1 | -360~360°/s | 뒷 휠 구동 |

## Step 4.2: IsaacSim Action Graph 설정

1. **IsaacSim 실행 및 로봇 로드**
   ```bash
   conda activate isaac_sim
   isaacsim
   ```

2. **Action Graph 창 열기**
   - 메뉴: `Window` → `Action Graph`

3. **ROS2 브릿지 활성화**
   - 메뉴: `Extensions` → `ROS2 Bridge`
   - ROS2 Bridge가 활성화되면 다양한 ROS2 노드 사용 가능

4. **Action Graph 노드 추가**

   a. **ROS2 Subscribe Joint State 노드**
   - Action Graph 우클릭 → `Add Node`
   - 검색: `ROS2 Subscribe Joint State`
   - 추가

   b. **ArticulationController 노드**
   - Action Graph 우클릭 → `Add Node`
   - 검색: `ArticulationController`
   - 추가

5. **노드 연결**
   - ROS2 Subscribe Joint State의 출력 → ArticulationController의 입력 연결
   - Topic 이름 설정: `/isaac_sim/joint_commands`

6. **Articulation 대상 설정**
   - ArticulationController 선택
   - Properties: Articulation Prim Path → 로봇 경로 설정
     ```
     /World/ffw_sg2_follower  (또는 실제 로봇 경로)
     ```

## Step 4.3: 관절 명령 발행 Python 코드

ROS2에서 관절 명령을 발행하는 Python 스크립트를 작성합니다.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class FFWSg2Controller(Node):
    def __init__(self):
        super().__init__('ffw_sg2_controller')

        # Joint State 발행자 생성
        self.joint_publisher = self.create_publisher(
            JointState,
            '/isaac_sim/joint_commands',
            10
        )

        # 타이머: 100Hz (0.01초)
        self.timer = self.create_timer(0.01, self.send_joint_commands)

        # 시간 추적
        self.start_time = time.time()

        # 관절 이름 정의
        self.joint_names = [
            # 팔 (7-DOF x 2)
            'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
            'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
            'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
            'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
            # 머리 (1 DOF x 2)
            'head_joint1', 'head_joint2',
            # 그리퍼 (1 제어 입력 x 2)
            'gripper_l_joint1', 'gripper_r_joint1',
            # 리프트
            'lift_joint',
            # 휠 스티어 및 구동
            'left_wheel_steer', 'left_wheel_drive',
            'right_wheel_steer', 'right_wheel_drive',
            'rear_wheel_steer', 'rear_wheel_drive'
        ]

    def send_joint_commands(self):
        """정현파를 사용하여 관절 명령 발행"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # 경과 시간
        elapsed = time.time() - self.start_time

        # 정현파 계산 (T = 4초 주기)
        positions = []
        for i, joint_name in enumerate(self.joint_names):
            # 각 관절마다 약간 다른 위상 추가
            phase = (2 * math.pi * i) / len(self.joint_names)

            if 'arm' in joint_name:
                # 팔 관절: ±0.5 라디안 범위
                pos = 0.5 * math.sin(2 * math.pi * elapsed / 4.0 + phase)
            elif 'head' in joint_name:
                # 머리 관절: ±0.3 라디안 범위
                pos = 0.3 * math.sin(2 * math.pi * elapsed / 4.0 + phase)
            elif 'gripper' in joint_name:
                # 그리퍼: 0~1 범위로 정규화
                pos = 0.5 + 0.5 * math.sin(2 * math.pi * elapsed / 4.0 + phase)
            elif 'lift' in joint_name:
                # 리프트: 0.25~0.50 범위 (미터 단위, 250~500mm)
                pos = 0.375 + 0.125 * math.sin(2 * math.pi * elapsed / 4.0 + phase)
            elif 'steer' in joint_name:
                # 휠 스티어: ±45° (±0.785 라디안)
                pos = 0.785 * math.sin(2 * math.pi * elapsed / 4.0 + phase)
            else:  # 휠 구동
                # 휠 구동: ±2 rad/s
                pos = 2.0 * math.sin(2 * math.pi * elapsed / 4.0 + phase)

            positions.append(pos)

        msg.position = positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.joint_publisher.publish(msg)
        self.get_logger().debug(f"Published joint commands: {len(positions)} joints")

def main(args=None):
    rclpy.init(args=args)
    controller = FFWSg2Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 4.4: Python 스크립트 실행

### 4.4.1 ROS2 환경 설정

```bash
# ROS2 Jazzy 환경 활성화
source /opt/ros/jazzy/setup.bash

# 워크스페이스 설정 (설치된 경우)
source /home/cho/ms_AIworker/isaacsim_ai_worker/ros2_ws/install/setup.bash
```

### 4.4.2 스크립트 저장 및 실행

```bash
# 스크립트 저장 위치
mkdir -p /home/cho/ms_AIworker/scripts

# 스크립트 저장
cat > /home/cho/ms_AIworker/scripts/isaac_sim_control.py << 'EOF'
# 위의 Python 코드를 여기에 붙여넣기
EOF

# 스크립트 실행 권한 부여
chmod +x /home/cho/ms_AIworker/scripts/isaac_sim_control.py

# 실행 (IsaacSim이 동시에 실행 중이어야 함)
python3 /home/cho/ms_AIworker/scripts/isaac_sim_control.py
```

### 4.4.3 IsaacSim에서 시뮬레이션 시작

1. IsaacSim 윈도우에서 **Play 버튼 클릭** (시뮬레이션 시작)
2. Python 스크립트 실행 (별도 터미널에서)
3. 로봇의 관절이 정현파에 따라 움직이는지 확인

## Step 4.5: 관절 제어 검증

### 4.5.1 ROS2 Topic 모니터링

다른 터미널에서 발행된 Topic 확인:

```bash
# ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash

# 발행된 Topic 목록 확인
ros2 topic list

# Joint State 메시지 확인
ros2 topic echo /isaac_sim/joint_commands

# 메시지 발행 빈도 확인
ros2 topic hz /isaac_sim/joint_commands
```

### 4.5.2 Rviz2에서 시각화

```bash
# Rviz2 실행
rviz2

# 로봇 모델 표시
# - Fixed Frame: world
# - Add → RobotModel
# - RobotModel의 Urdf/Sdf Url: ffw_sg2_follower.urdf 경로 설정
```

## Step 4.6: 맞춤형 제어 구현

정현파 테스트 대신 자신의 제어 알고리즘을 구현할 수 있습니다:

```python
def send_joint_commands(self):
    """자신의 제어 로직 구현"""
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = self.joint_names

    # 예: 특정 관절 조합으로 제어
    positions = [
        0.0,  # arm_l_joint1
        -1.57,  # arm_l_joint2 (약 -90도)
        # ... 다른 관절들
    ]

    msg.position = positions
    self.joint_publisher.publish(msg)
```

## Troubleshooting

### ROS2 토픽 발행 실패
- ROS2 환경이 활성화되어 있는지 확인: `echo $ROS_DISTRO`
- IsaacSim ROS2 Bridge가 활성화되어 있는지 확인
- 메뉴: `Extensions` → `ROS2 Bridge`

### 관절이 움직이지 않음
- Action Graph가 올바르게 연결되었는지 확인
- ArticulationController 노드의 Prim Path가 올바른지 확인
- IsaacSim 시뮬레이션이 재생 중(Play)인지 확인

### 관절 명령이 이상함
- 관절 이름이 URDF와 일치하는지 확인
- 관절 범위가 올바른지 확인
- Python 스크립트의 로깅 확인: `rclpy.get_logger().info()`

### 성능 문제
- 발행 빈도를 줄이기: `self.create_timer(0.05, ...)` (20Hz로 변경)
- IsaacSim 그래픽 설정 저하: Edit → Preferences → Rendering

---
**Status**: PENDING
**Next**: [Step 5: Swerve Drive](05-swerve-drive.md)
