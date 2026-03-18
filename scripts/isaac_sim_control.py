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
        arm_amp_rad = math.radians(15.0)
        arm_freq = 0.5
        arm_val1 = arm_amp_rad * math.sin(2.0 * math.pi * arm_freq * t)
        arm_val2 = -arm_amp_rad * math.sin(2.0 * math.pi * arm_freq * t)
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
        positions['gripper_r_joint1'] = math.radians(grip_r_val)

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
