# Step 5: Control Humanoids

## Overview
ROS2를 통해 IsaacSim 상의 FFW-SG2 로봇 관절을 제어합니다.

1. JointState 토픽 개념 이해
2. IsaacSim Action Graph에서 ROS2 Joint State 구독 설정
3. Python 스크립트로 관절 명령 발행 & 테스트

## Prerequisites
- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 완료 (Step 2)
- [x] 센서 구성 완료 (Step 3)
- [x] TF 발행 완료 (Step 4)
- [x] ROS2 Jazzy 설치 완료

---

## [1] JointState 토픽 개념

### sensor_msgs/JointState 메시지 구조

ROS2에서 로봇 관절을 제어할 때 사용하는 표준 메시지입니다.

```
header:
  stamp: {sec: 1234, nanosec: 567}    # 타임스탬프
name:      ['arm_l_joint1', 'head_joint1', ...]  # 관절 이름 (배열)
position:  [0.5,             0.3,          ...]  # 목표 위치 (라디안)
velocity:  [0.0,             0.0,          ...]  # 목표 속도 (rad/s)
effort:    [0.0,             0.0,          ...]  # 목표 힘/토크 (Nm)
```

- **name과 position은 1:1 매칭**: `name[0]`의 목표가 `position[0]`
- **모든 관절을 한 번에 보낼 필요 없음**: 팔만 움직이고 싶으면 팔 관절만 넣으면 됨
- **단위는 라디안**: 30도 = 약 0.523 rad (`math.radians(30)`)

### 3가지 제어 모드

| 모드 | 배열 | 예시 | 용도 |
|------|------|------|------|
| **Position Control** (위치) | `position` | "팔을 30도로 이동" | 가장 기본적이고 안전한 방식 |
| **Velocity Control** (속도) | `velocity` | "초당 1rad으로 회전" | 바퀴 구동에 주로 사용 |
| **Effort Control** (힘/토크) | `effort` | "10Nm 토크 적용" | 힘 제어, Admittance Control |

> **참고**: IsaacSim의 Articulation Controller는 position, velocity, effort 중
> 값이 들어있는 것을 자동 감지하여 해당 모드로 제어합니다.

### 동작 흐름

```
호스트 (Python 스크립트)                    IsaacSim (컨테이너)

 JointState 메시지 생성                   JointControlGraph:
 - name: ['arm_l_joint1']
 - position: [0.523]                    ROS2 Subscribe Joint State
        │                                (토픽에서 관절 명령 수신)
        │  /isaac_sim/joint_commands          │
        └──────── ROS2 토픽 ──────────→       │
                                              ▼
                                         Articulation Controller
                                         (받은 명령대로 로봇 관절 구동)
```

> **연속 발행이 중요**: 한 번만 보내면 한 번만 움직입니다.
> 보통 20~100Hz로 계속 발행하여 부드러운 모션을 구현합니다.

---

## [2] 로봇 관절 구성

### 팔 관절 (7-DOF x 2)

| ID | 관절명 | Technical Name | 범위 | 설명 |
|----|--------|---------------|------|------|
| 1 | Right Shoulder Pitch | arm_r_joint1 | -180° ~ 180° | |
| 2 | Right Shoulder Roll | arm_r_joint2 | -190° ~ 10° | 비대칭 범위 |
| 3 | Right Shoulder Yaw | arm_r_joint3 | -180° ~ 180° | |
| 4 | Right Elbow | arm_r_joint4 | -170° ~ 65° | 비대칭 범위 |
| 5 | Right Wrist Yaw | arm_r_joint5 | -180° ~ 180° | |
| 6 | Right Wrist Pitch | arm_r_joint6 | -105° ~ 105° | |
| 7 | Right Wrist Roll | arm_r_joint7 | -120° ~ 90° | 비대칭 범위 |
| 31 | Left Shoulder Pitch | arm_l_joint1 | -180° ~ 180° | |
| 32 | Left Shoulder Roll | arm_l_joint2 | -10° ~ 190° | 오른팔과 반대 |
| 33 | Left Shoulder Yaw | arm_l_joint3 | -180° ~ 180° | |
| 34 | Left Elbow | arm_l_joint4 | -170° ~ 65° | |
| 35 | Left Wrist Yaw | arm_l_joint5 | -180° ~ 180° | |
| 36 | Left Wrist Pitch | arm_l_joint6 | -105° ~ 105° | |
| 37 | Left Wrist Roll | arm_l_joint7 | -90° ~ 120° | 오른팔과 반대 |

### 머리 관절

| ID | 관절명 | Technical Name | 범위 |
|----|--------|---------------|------|
| 61 | Head Pitch | head_joint1 | -50° ~ 30° |
| 62 | Head Yaw | head_joint2 | -20° ~ 20° |

### 리프트

| ID | 관절명 | Technical Name | 범위 |
|----|--------|---------------|------|
| 81 | Lift | lift_joint | 0 ~ 500 mm |

### 그리퍼 (RH-P12-RN)

| ID | 관절명 | Technical Name | 범위 |
|----|--------|---------------|------|
| 8 | Right Gripper | gripper_r_joint1 | 0 ~ 107.6 mm |
| 38 | Left Gripper | gripper_l_joint1 | 0 ~ 107.6 mm |

> **그리퍼 이슈**: IsaacSim에서 그리퍼당 4개 Joint(gripper_l_joint1~4)을 제어할 수 있지만,
> 실제 **제어 입력은 1개(1-DOF)**입니다. 1개의 입력값으로 나머지 3개 Joint 값을
> 그리퍼 kinematics에 맞게 계산해야 합니다.
>
> **RH-P12-RN 스펙**: 무게 500g, 최대 파지력 5kg, 10W DC 모터,
> 토크 제어 및 전류 기반 위치 제어 지원, 교체 가능한 핑거팁

### 모바일 베이스 (Swerve Drive)

| 관절명 | Technical Name | 범위 |
|--------|---------------|------|
| Right Wheel Steer | right_wheel_steer | -90° ~ 90° |
| Left Wheel Steer | left_wheel_steer | -90° ~ 90° |
| Rear Wheel Steer | rear_wheel_steer | -90° ~ 90° |
| Right Wheel Drive | right_wheel_drive | -360° ~ 360° |
| Left Wheel Drive | left_wheel_drive | -360° ~ 360° |
| Rear Wheel Drive | rear_wheel_drive | -360° ~ 360° |

---

## [3] IsaacSim Action Graph 설정

### 방법 1: Python Script Editor (권장)

GUI Action Graph 에디터는 IsaacSim 5.1.0에서 **프리징 버그**가 있습니다 (GitHub [#329](https://github.com/isaac-sim/IsaacSim/issues/329), [#490](https://github.com/isaac-sim/IsaacSim/issues/490)).
Python Script Editor로 노드를 추가하면 프리징 없이 안전하게 생성할 수 있습니다.

1. `Window` → `Script Editor` 열기
2. 아래 코드를 붙여넣고 **Run** 클릭:

```python
import omni.graph.core as og
keys = og.Controller.Keys
og.Controller.edit({"graph_path": "/JointControlGraph", "evaluator_name": "execution"}, {keys.CREATE_NODES: [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"), ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"), ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"), ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController")], keys.SET_VALUES: [("SubscribeJointState.inputs:topicName", "/isaac_sim/joint_commands"), ("ArticulationController.inputs:targetPrim", "/ffw_sg2_follower/world")], keys.CONNECT: [("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"), ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"), ("ROS2Context.outputs:context", "SubscribeJointState.inputs:context"), ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"), ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"), ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"), ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand")]})
print("Joint Control Action Graph created!")
```

> `Joint Control Action Graph created!` 메시지가 나오면 성공입니다.

### 방법 2: GUI Action Graph 에디터

프리징이 발생하지 않는 경우 GUI로도 가능합니다.

1. `Window` → `Graph Editors` → `Action Graph` → **New Action Graph**
2. 노드 추가 (우클릭 → 검색):
   - **On Playback Tick**
   - **ROS2 Context**
   - **ROS2 Subscribe Joint State**
   - **Articulation Controller**

3. 노드 연결:

```
On Playback Tick [Tick] ──→ ROS2 Subscribe Joint State [ExecIn]
ROS2 Subscribe Joint State [ExecOut] ──→ Articulation Controller [ExecIn]

ROS2 Context [Context] ──→ ROS2 Subscribe Joint State [Context]

ROS2 Subscribe Joint State [Joint Names] ──→ Articulation Controller [Joint Names]
ROS2 Subscribe Joint State [Position Command] ──→ Articulation Controller [Position Command]
ROS2 Subscribe Joint State [Velocity Command] ──→ Articulation Controller [Velocity Command]
ROS2 Subscribe Joint State [Effort Command] ──→ Articulation Controller [Effort Command]
```

4. 속성 설정:

| 노드 | 항목 | 값 |
|------|------|-----|
| ROS2 Subscribe Joint State | topicName | `/isaac_sim/joint_commands` |
| Articulation Controller | targetPrim | `ffw_sg2_follower > world` |

> **targetPrim 주의**: `ffw_sg2_follower` 자체가 아니라 **`ffw_sg2_follower > world`**를 선택해야 합니다.
> `world`에 Articulation Root가 있습니다 (Step 4 참고).

---

## [4] 관절 제어 Python 스크립트

호스트에서 ROS2 토픽을 발행하여 로봇을 제어하는 스크립트입니다.
정현파(sine wave)로 관절을 움직여 모든 관절이 정상 동작하는지 검증합니다.

### 스크립트 실행 환경

```bash
# 호스트 터미널에서 (Docker 환경은 ros2-bridge-env.sh 사용)
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
```

### 테스트 코드

```python
#!/usr/bin/env python3
"""FFW-SG2 관절 정현파 테스트 - 모든 관절을 sine wave로 구동하여 검증"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class IsaacSimJointController(Node):
    def __init__(self):
        super().__init__('isaac_sim_joint_controller')

        self.publisher_ = self.create_publisher(
            JointState, '/isaac_sim/joint_commands', 10
        )

        # 20Hz (0.05초) 주기로 발행
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info('Isaac Sim Joint Controller started.')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        t = time.time() - self.start_time

        # 관절 이름 정의 (IsaacSim 순서)
        msg.name = []
        msg.name.extend(['left_wheel_steer', 'left_wheel_drive'])
        msg.name.append('lift_joint')
        msg.name.extend([f'arm_l_joint{i}' for i in range(1, 8)])
        msg.name.extend([f'gripper_l_joint{i}' for i in range(1, 5)])
        msg.name.extend([f'arm_r_joint{i}' for i in range(1, 8)])
        msg.name.extend([f'gripper_r_joint{i}' for i in range(1, 5)])
        msg.name.extend(['head_joint1', 'head_joint2'])
        msg.name.extend([
            'rear_wheel_steer', 'rear_wheel_drive',
            'right_wheel_steer', 'right_wheel_drive'
        ])

        # 각 관절별 목표값 계산
        positions = {name: 0.0 for name in msg.name}

        # 1. 팔 관절: 진폭 15도, 0.5Hz
        arm_amp = 15.0  # degrees
        arm_freq = 0.5
        arm_val1 = arm_amp * math.sin(2.0 * math.pi * arm_freq * t)
        arm_val2 = -arm_amp * math.sin(2.0 * math.pi * arm_freq * t)
        positions['arm_l_joint1'] = arm_val1
        positions['arm_r_joint1'] = arm_val2

        # 2. 리프트: -0.2 ~ -0.1 범위
        lift_center = -0.15
        lift_amp = 0.01
        lift_freq = 0.1
        positions['lift_joint'] = lift_center + lift_amp * math.sin(
            2.0 * math.pi * lift_freq * t
        )

        # 3. 왼쪽 그리퍼: 20~40도 범위
        grip_l_center = 30.0
        grip_l_amp = 10.0
        grip_l_freq = 1.0
        grip_l_val = grip_l_center + grip_l_amp * math.sin(
            2.0 * math.pi * grip_l_freq * t
        )
        positions['gripper_l_joint1'] = math.radians(grip_l_val)

        # 4. 오른쪽 그리퍼: 20~40도 범위
        grip_r_center = 30.0
        grip_r_amp = 10.0
        grip_r_freq = 1.0
        grip_r_val = grip_r_center + grip_r_amp * math.sin(
            2.0 * math.pi * grip_r_freq * t
        )
        positions['gripper_r_joint2'] = grip_r_val

        # position 배열 생성
        msg.position = [positions[name] for name in msg.name]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 실행

```bash
# 1. IsaacSim에서 Play(▶) 클릭 (먼저!)
# 2. 호스트 터미널에서:
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
python3 ~/ms_AIworker/scripts/isaac_sim_control.py
```

로봇의 팔, 머리, 리프트, 그리퍼가 정현파로 움직이면 성공입니다.

---

## [5] 테스트 & 검증

### 토픽 확인

```bash
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

# 토픽 목록 확인
ros2 topic list | grep joint

# 발행 주파수 확인 (20Hz 나와야 정상)
ros2 topic hz /isaac_sim/joint_commands

# 메시지 내용 확인
ros2 topic echo /isaac_sim/joint_commands --once
```

### RViz2에서 확인

```bash
rviz2
```

- Fixed Frame: `World`
- Add → **TF** (관절 움직임에 따라 frame이 변하는지 확인)

---

## Troubleshooting

### Action Graph 에디터 프리징 (GUI 멈춤)

IsaacSim 5.1.0의 확인된 버그입니다 (GitHub [#329](https://github.com/isaac-sim/IsaacSim/issues/329), [#490](https://github.com/isaac-sim/IsaacSim/issues/490)).
→ **Python Script Editor로 노드 추가** ([3] 방법 1 참고)

### 관절이 움직이지 않음

- IsaacSim이 **Play 상태**인지 확인
- `ros2 topic echo /isaac_sim/joint_commands`로 메시지가 오는지 확인
- Articulation Controller의 **targetPrim**이 `ffw_sg2_follower > world`인지 확인 (Articulation Root 위치)
- 관절 이름이 URDF와 정확히 일치하는지 확인

### Python 스크립트에서 토픽이 안 보임

- `source ~/ms_AIworker/scripts/ros2-bridge-env.sh` 실행했는지 확인
- Docker 컨테이너가 `network_mode: host`로 실행 중인지 확인
- FastDDS UDP 설정이 호스트/컨테이너 양쪽에 적용되어 있는지 확인

### 특정 관절만 안 움직임

- 해당 관절의 Joint Drive가 설정되어 있는지 IsaacSim Stage에서 확인
- URDF 임포트 시 Joint Drive 설정이 누락될 수 있음
- Stage에서 해당 joint 클릭 → Property → **Drive** 항목 확인

### 그리퍼가 이상하게 움직임

- 그리퍼는 4개 Joint이지만 **1-DOF 제어 입력**
- `gripper_l_joint1` 값으로 나머지 joint2~4의 값을 kinematics로 계산해야 함
- 단순히 joint1만 제어하면 물리적으로 비현실적인 동작이 발생할 수 있음

---
**Status**: COMPLETED
**이전**: [Step 4: Publish TF Tree](04-publish-tf.md)
**다음**: [Step 6: Swerve Drive](06-swerve-drive.md)
