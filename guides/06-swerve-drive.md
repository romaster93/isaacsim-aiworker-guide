# Step 5: Swerve Drive Control

## Overview
FFW-SG2의 3-wheel triangular swerve drive 시스템을 제어합니다.
- 3개의 독립적인 스위브 모듈 (왼쪽, 오른쪽, 뒤)
- 각 휠: 독립 스티어 + 독립 구동
- 전진/역진 운동학(Forward/Inverse Kinematics)
- 속도 명령 및 휠 각도 계산

## Prerequisites
- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 (Step 2)
- [x] 관절 제어 기본 (Step 4)
- [x] ROS2 Jazzy 환경

## Step 5.1: Swerve Drive 기하학

FFW-SG2의 swerve 드라이브 배치:

```
      Front (Y+)

    Left (L)   Right (R)
        \     /
         \   /
          \ /
    -------+------- (X축)
          / \
         /   \
        /     \
    Rear (B)

각도 정의:
- Left (L): 120° (2π/3 라디안)
- Right (R): -120° (-2π/3 라디안)  또는 240°
- Rear (B): 0° (직선)

좌표계:
- X축: 로봇 전진 방향
- Y축: 로봇 좌측
- θ: 로봇 회전 (반시계방향 양수)
```

## Step 5.2: 운동학 공식

### 5.2.1 역운동학 (Inverse Kinematics)

로봇 속도 (vx, vy, ω)에서 휠 속도 및 각도 계산:

```python
import numpy as np
import math

class SwerveKinematics:
    def __init__(self, wheel_positions):
        """
        wheel_positions: 3개 휠의 위치 정보
        [
            {'name': 'left', 'x': -0.1, 'y': 0.15, 'angle': 2*pi/3},
            {'name': 'right', 'x': -0.1, 'y': -0.15, 'angle': -2*pi/3},
            {'name': 'rear', 'x': 0.2, 'y': 0, 'angle': 0}
        ]
        """
        self.wheels = wheel_positions
        self.num_wheels = len(wheel_positions)

    def inverse_kinematics(self, vx, vy, omega):
        """
        로봇 속도 (vx, vy, omega)를 휠 속도 및 각도로 변환

        입력:
        - vx: 전진 속도 (m/s)
        - vy: 좌측 속도 (m/s)
        - omega: 회전 각속도 (rad/s)

        출력:
        - wheel_velocities: 각 휠의 선형 속도 (m/s)
        - wheel_angles: 각 휠의 방향 (라디안)
        """
        wheel_velocities = []
        wheel_angles = []

        for wheel in self.wheels:
            # 휠 위치에서의 회전으로 인한 속도
            rx = wheel['x']
            ry = wheel['y']

            # 회전 성분: ω × r
            vx_rot = -omega * ry
            vy_rot = omega * rx

            # 총 속도
            wheel_vx = vx + vx_rot
            wheel_vy = vy + vy_rot

            # 속도 크기
            velocity = math.sqrt(wheel_vx**2 + wheel_vy**2)

            # 속도 방향
            if velocity > 0.01:  # 작은 속도 임계값
                angle = math.atan2(wheel_vy, wheel_vx)
            else:
                angle = wheel['angle']  # 정지 시 기본 각도

            wheel_velocities.append(velocity)
            wheel_angles.append(angle)

        return wheel_velocities, wheel_angles

    def forward_kinematics(self, wheel_velocities, wheel_angles):
        """
        휠 속도 및 각도에서 로봇 속도 계산 (역함수)

        입력:
        - wheel_velocities: 각 휠의 선형 속도
        - wheel_angles: 각 휠의 방향

        출력:
        - vx, vy, omega: 로봇 속도
        """
        # 각 휠의 속도 벡터
        vx_total = 0.0
        vy_total = 0.0
        omega_total = 0.0

        for i, wheel in enumerate(self.wheels):
            vel = wheel_velocities[i]
            angle = wheel_angles[i]

            # 속도 성분
            vx = vel * math.cos(angle)
            vy = vel * math.sin(angle)

            # 누적
            vx_total += vx
            vy_total += vy

            # 회전 성분 (ω = (r × v) / |r|²)
            rx = wheel['x']
            ry = wheel['y']
            r_squared = rx**2 + ry**2

            if r_squared > 0.001:
                omega_total += (rx * vy - ry * vx) / r_squared

        # 평균
        vx = vx_total / self.num_wheels
        vy = vy_total / self.num_wheels
        omega = omega_total / self.num_wheels

        return vx, vy, omega
```

## Step 5.3: Swerve Drive ROS2 제어 노드

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import time

class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')

        # 휠 위치 정의 (로봇 좌표계)
        self.wheel_positions = [
            {
                'name': 'left',
                'x': -0.1,
                'y': 0.15,
                'steer_joint': 'left_wheel_steer',
                'drive_joint': 'left_wheel_drive',
                'angle': 2 * math.pi / 3  # 120도
            },
            {
                'name': 'right',
                'x': -0.1,
                'y': -0.15,
                'steer_joint': 'right_wheel_steer',
                'drive_joint': 'right_wheel_drive',
                'angle': -2 * math.pi / 3  # -120도
            },
            {
                'name': 'rear',
                'x': 0.2,
                'y': 0,
                'steer_joint': 'rear_wheel_steer',
                'drive_joint': 'rear_wheel_drive',
                'angle': 0  # 0도 (뒤쪽)
            }
        ]

        # Joint State 발행자
        self.joint_publisher = self.create_publisher(
            JointState,
            '/isaac_sim/joint_commands',
            10
        )

        # Cmd_vel 구독자
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 현재 속도 명령 저장
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # 타이머: 100Hz
        self.timer = self.create_timer(0.01, self.publish_commands)

        self.get_logger().info("Swerve Drive Controller initialized")

    def cmd_vel_callback(self, msg: Twist):
        """Twist 메시지 수신 콜백"""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z

        self.get_logger().debug(
            f"Received velocity: vx={self.vx:.2f}, vy={self.vy:.2f}, "
            f"omega={self.omega:.2f}"
        )

    def inverse_kinematics(self):
        """운동학 계산"""
        wheel_velocities = []
        wheel_angles = []

        for wheel in self.wheel_positions:
            # 회전으로 인한 속도
            rx = wheel['x']
            ry = wheel['y']

            vx_rot = -self.omega * ry
            vy_rot = self.omega * rx

            # 총 속도
            wheel_vx = self.vx + vx_rot
            wheel_vy = self.vy + vy_rot

            # 속도 크기 및 방향
            velocity = math.sqrt(wheel_vx**2 + wheel_vy**2)

            if velocity > 0.01:
                angle = math.atan2(wheel_vy, wheel_vx)
            else:
                angle = wheel['angle']

            wheel_velocities.append(velocity)
            wheel_angles.append(angle)

        return wheel_velocities, wheel_angles

    def publish_commands(self):
        """관절 명령 발행"""
        wheel_velocities, wheel_angles = self.inverse_kinematics()

        # Joint State 메시지 생성
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        # 휠 명령 추가
        for i, wheel in enumerate(self.wheel_positions):
            # Steer 관절
            msg.name.append(wheel['steer_joint'])
            msg.position.append(wheel_angles[i])
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

            # Drive 관절
            msg.name.append(wheel['drive_joint'])
            msg.position.append(0.0)  # Position이 아닌 velocity로 제어
            msg.velocity.append(wheel_velocities[i])
            msg.effort.append(0.0)

        self.joint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SwerveController()

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

## Step 5.4: Swerve Drive 테스트

### 5.4.1 스크립트 저장 및 실행

```bash
# 스크립트 저장
cat > /home/cho/ms_AIworker/scripts/swerve_controller.py << 'EOF'
# 위의 Python 코드를 여기에 붙여넣기
EOF

# 실행 권한 부여
chmod +x /home/cho/ms_AIworker/scripts/swerve_controller.py

# 실행
source /opt/ros/jazzy/setup.bash
python3 /home/cho/ms_AIworker/scripts/swerve_controller.py
```

### 5.4.2 속도 명령 발행

다른 터미널에서:

```bash
# ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash

# 전진 (vx = 0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

# 좌측 이동 (vy = 0.3 m/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0.3, z: 0}, angular: {x: 0, y: 0, z: 0}}'

# 회전 (omega = 0.5 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'

# 복합 이동
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3, y: 0.2, z: 0}, angular: {x: 0, y: 0, z: 0.3}}'
```

## Step 5.5: 텔레옵 제어 (Keyboard)

keyboard를 사용하여 로봇을 제어합니다:

```bash
# teleop_twist_keyboard 설치 (존재하지 않으면)
sudo apt-get install ros-jazzy-teleop-twist-keyboard

# 실행
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

키보드 제어:
- `U` / `O`: 좌측 회전
- `I`: 전진
- `K`: 정지
- `,` / `.`: 우측 회전
- `J`: 후진
- `<` / `>`: 회전 속도 증가/감소
- `W` / `Z`: 선형 속도 증가/감소

## Step 5.6: 속도 제한 및 최적화

### 5.6.1 최대 속도 설정

```python
class SwerveController(Node):
    def __init__(self):
        # ... 기존 초기화 코드 ...

        # 최대 속도 제한
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 2.0  # rad/s
        self.max_wheel_velocity = 2.0  # m/s

    def cmd_vel_callback(self, msg: Twist):
        """Twist 메시지 수신 (속도 제한 적용)"""
        # 선형 속도 제한
        vx = msg.linear.x
        vy = msg.linear.y
        linear_mag = math.sqrt(vx**2 + vy**2)

        if linear_mag > self.max_linear_velocity:
            scale = self.max_linear_velocity / linear_mag
            vx *= scale
            vy *= scale

        # 각속도 제한
        omega = msg.angular.z
        if abs(omega) > self.max_angular_velocity:
            omega = math.copysign(self.max_angular_velocity, omega)

        self.vx = vx
        self.vy = vy
        self.omega = omega
```

### 5.6.2 휠 속도 정규화

```python
def publish_commands(self):
    """관절 명령 발행 (정규화 포함)"""
    wheel_velocities, wheel_angles = self.inverse_kinematics()

    # 최대 휠 속도 확인
    max_velocity = max(wheel_velocities) if wheel_velocities else 0.0

    # 초과 시 정규화
    if max_velocity > self.max_wheel_velocity:
        wheel_velocities = [v * (self.max_wheel_velocity / max_velocity)
                           for v in wheel_velocities]

    # ... 나머지 코드 ...
```

## Troubleshooting

### 로봇이 예상대로 움직이지 않음
- 휠 위치 정보 (x, y) 확인
- 휠 기본 각도 확인
- 운동학 계산 결과 로깅: `self.get_logger().info(...)`

### 휠이 진동함
- 속도 명령이 너무 높지 않은지 확인
- PID 게인 조정 (IsaacSim 또는 로봇 제어기)
- 샘플링 시간 증가 (0.01 → 0.05초)

### ROS2 토픽 발행 실패
- ROS2 브릿지 활성화 확인
- Topic 이름 일치 확인: `/cmd_vel`, `/isaac_sim/joint_commands`

### 로봇이 미끄러짐
- 마찰 계수 조정
- 접지 조건 확인

---
**Status**: PENDING
**Next**: [Step 6: Kinematic Override Drive](06-kinematic-override.md)
