#!/usr/bin/env python3
"""FFW-SG2 Swerve Drive Controller
/cmd_vel (Twist) → 역운동학 → /isaac_sim/joint_commands (JointState)

Steer: position control (목표 각도)
Drive: velocity control (목표 각속도)

사전 설정 필요 (IsaacSim Script Editor에서 실행):
- Drive wheel: Stiffness=0, Damping=MAX, Limit=Not Limited
- Steer wheel: Stiffness=MAX, Damping=0, Limit=-90~+90
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math


class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')

        # 휠 위치 정의 (URDF 실측값, 로봇 좌표계)
        self.wheel_positions = [
            {
                'name': 'left',
                'x': 0.148, 'y': 0.236,
                'steer_joint': 'left_wheel_steer',
                'drive_joint': 'left_wheel_drive',
            },
            {
                'name': 'right',
                'x': 0.150, 'y': -0.273,
                'steer_joint': 'right_wheel_steer',
                'drive_joint': 'right_wheel_drive',
            },
            {
                'name': 'rear',
                'x': -0.278, 'y': -0.021,
                'steer_joint': 'rear_wheel_steer',
                'drive_joint': 'rear_wheel_drive',
            }
        ]

        self.wheel_radius = 0.033  # 바퀴 반지름 (m), PDF params.yaml 참고
        self.max_wheel_vel = 10.0  # 최대 바퀴 각속도 (rad/s)

        self.joint_publisher = self.create_publisher(
            JointState, '/isaac_sim/joint_commands', 10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        self.create_timer(0.02, self.publish_commands)  # 50Hz
        self.get_logger().info('Swerve Drive Controller started.')

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def inverse_kinematics(self):
        wheel_velocities = []
        wheel_angles = []

        for wheel in self.wheel_positions:
            rx = wheel['x']
            ry = wheel['y']

            # 회전으로 인한 속도 성분
            vx_rot = -self.omega * ry
            vy_rot = self.omega * rx

            # 총 속도
            wheel_vx = self.vx + vx_rot
            wheel_vy = self.vy + vy_rot

            speed = math.sqrt(wheel_vx**2 + wheel_vy**2)

            if speed > 0.01:
                angle = math.atan2(wheel_vy, wheel_vx)
                # 180도 플립 최적화: steer 범위를 -90~90도로 유지
                if angle > math.pi / 2:
                    angle -= math.pi
                    speed = -speed
                elif angle < -math.pi / 2:
                    angle += math.pi
                    speed = -speed
            else:
                angle = 0.0
                speed = 0.0

            # m/s → rad/s 변환
            angular_vel = speed / self.wheel_radius

            wheel_velocities.append(angular_vel)
            wheel_angles.append(angle)

        # 속도 제한
        max_vel = max(abs(v) for v in wheel_velocities) if wheel_velocities else 0
        if max_vel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_vel
            wheel_velocities = [v * scale for v in wheel_velocities]

        return wheel_velocities, wheel_angles

    def publish_commands(self):
        # cmd_vel 타임아웃
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.omega = 0.0

        wheel_velocities, wheel_angles = self.inverse_kinematics()

        # Steer + Drive를 하나의 메시지로 (position 비움으로 drive 잠김 방지)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []

        # Steer joints: position control
        for i, wheel in enumerate(self.wheel_positions):
            msg.name.append(wheel['steer_joint'])
            msg.position.append(wheel_angles[i])
            msg.velocity.append(0.0)

        # Drive joints: velocity control (position 없이 — NaN으로)
        for i, wheel in enumerate(self.wheel_positions):
            msg.name.append(wheel['drive_joint'])
            msg.position.append(float('nan'))
            msg.velocity.append(wheel_velocities[i])

        self.joint_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
