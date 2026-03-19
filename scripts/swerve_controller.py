#!/usr/bin/env python3
"""FFW-SG2 Swerve Drive Controller + Odometry
/cmd_vel (Twist) → 역운동학 → /isaac_sim/joint_commands (JointState)
/joint_states → 정운동학(FK) → /odom (Odometry)

IK: cmd_vel → 바퀴 명령 (제어)
FK: 바퀴 상태 → 로봇 속도 → 위치 적분 (odometry)

ROBOTIS ffw_swerve_drive_controller의 odometry.cpp와 동일한 방식:
- 각 바퀴의 steer 각도 + drive 각속도 → 최소제곱법으로 로봇 속도 추정
- 속도를 시간 적분하여 위치 계산

사전 설정 필요 (IsaacSim Script Editor에서 실행):
- Drive wheel: Stiffness=0, Damping=MAX, Limit=Not Limited
- Steer wheel: Stiffness=MAX, Damping=0, Limit=-90~+90
- Joint State Publisher: /joint_states 토픽 발행 설정
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


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

        self.wheel_radius = 0.033  # 바퀴 반지름 (m)
        self.max_wheel_vel = 10.0  # 최대 바퀴 각속도 (rad/s)
        self.num_wheels = len(self.wheel_positions)

        # === IK: cmd_vel → joint commands ===
        self.joint_publisher = self.create_publisher(
            JointState, '/isaac_sim/joint_commands', 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        # === FK: joint_states → odometry ===
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_states_callback, sensor_qos
        )

        # Odometry 상태
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_omega = 0.0
        self.last_odom_time = None

        # Odometry publisher (메시지만, TF는 nav2_bridge가 담당)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # IK 타이머 (50Hz)
        self.create_timer(0.02, self._publish_commands)

        self.get_logger().info(
            'Swerve Drive Controller started\n'
            '  - IK: /cmd_vel → /isaac_sim/joint_commands\n'
            '  - FK: /joint_states → /odom (encoder-based odometry)'
        )

    # ==================== IK (제어) ====================

    def _cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _inverse_kinematics(self):
        wheel_velocities = []
        wheel_angles = []

        for wheel in self.wheel_positions:
            rx = wheel['x']
            ry = wheel['y']

            vx_rot = -self.omega * ry
            vy_rot = self.omega * rx

            wheel_vx = self.vx + vx_rot
            wheel_vy = self.vy + vy_rot

            speed = math.sqrt(wheel_vx**2 + wheel_vy**2)

            if speed > 0.01:
                angle = math.atan2(wheel_vy, wheel_vx)
                if angle > math.pi / 2:
                    angle -= math.pi
                    speed = -speed
                elif angle < -math.pi / 2:
                    angle += math.pi
                    speed = -speed
            else:
                angle = 0.0
                speed = 0.0

            angular_vel = speed / self.wheel_radius
            wheel_velocities.append(angular_vel)
            wheel_angles.append(angle)

        max_vel = max(abs(v) for v in wheel_velocities) if wheel_velocities else 0
        if max_vel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_vel
            wheel_velocities = [v * scale for v in wheel_velocities]

        return wheel_velocities, wheel_angles

    def _publish_commands(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.omega = 0.0

        wheel_velocities, wheel_angles = self._inverse_kinematics()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []

        for i, wheel in enumerate(self.wheel_positions):
            msg.name.append(wheel['steer_joint'])
            msg.position.append(wheel_angles[i])
            msg.velocity.append(0.0)

        for i, wheel in enumerate(self.wheel_positions):
            msg.name.append(wheel['drive_joint'])
            msg.position.append(float('nan'))
            msg.velocity.append(wheel_velocities[i])

        self.joint_publisher.publish(msg)

    # ==================== FK (odometry) ====================

    def _joint_states_callback(self, msg: JointState):
        """Joint state에서 바퀴 상태 추출 → FK → odometry 발행"""
        # joint name → index 매핑
        try:
            name_to_idx = {name: i for i, name in enumerate(msg.name)}
        except Exception:
            return

        # 각 바퀴의 steer 각도, drive 각속도 추출
        steer_positions = []
        drive_velocities = []

        for wheel in self.wheel_positions:
            steer_idx = name_to_idx.get(wheel['steer_joint'])
            drive_idx = name_to_idx.get(wheel['drive_joint'])

            if steer_idx is None or drive_idx is None:
                return

            steer_positions.append(msg.position[steer_idx])
            drive_velocities.append(msg.velocity[drive_idx])

        # 시간 계산
        now = self.get_clock().now()
        if self.last_odom_time is None:
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt < 0.001 or dt > 0.5:
            self.last_odom_time = now
            return
        self.last_odom_time = now

        # Forward Kinematics — 최소제곱법 (ROBOTIS odometry.cpp와 동일)
        # 각 바퀴: vx_module = v_w * cos(θ), vy_module = v_w * sin(θ)
        # 방정식: vx - ly*ω = vx_module, vy + lx*ω = vy_module
        # 3바퀴 → 6방정식, 3미지수(vx, vy, ω) → SVD로 풀기
        A = np.zeros((2 * self.num_wheels, 3))
        b = np.zeros(2 * self.num_wheels)

        for i in range(self.num_wheels):
            theta_s = steer_positions[i]
            omega_w = drive_velocities[i]
            v_w = omega_w * self.wheel_radius

            lx = self.wheel_positions[i]['x']
            ly = self.wheel_positions[i]['y']

            vx_module = v_w * math.cos(theta_s)
            vy_module = v_w * math.sin(theta_s)

            row1 = i * 2
            row2 = row1 + 1

            # vx - ly*ω = vx_module
            A[row1, 0] = 1.0
            A[row1, 1] = 0.0
            A[row1, 2] = -ly
            b[row1] = vx_module

            # vy + lx*ω = vy_module
            A[row2, 0] = 0.0
            A[row2, 1] = 1.0
            A[row2, 2] = lx
            b[row2] = vy_module

        # SVD로 풀기 (ROBOTIS 기본값)
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        vx_base = result[0]
        vy_base = result[1]
        omega_base = result[2]

        # 속도 저장
        self.odom_vx = vx_base
        self.odom_vy = vy_base
        self.odom_omega = omega_base

        # 위치 적분 (오일러 방법)
        # base frame 속도 → world frame 속도
        cos_yaw = math.cos(self.odom_yaw)
        sin_yaw = math.sin(self.odom_yaw)

        vx_world = vx_base * cos_yaw - vy_base * sin_yaw
        vy_world = vx_base * sin_yaw + vy_base * cos_yaw

        self.odom_x += vx_world * dt
        self.odom_y += vy_world * dt
        self.odom_yaw += omega_base * dt

        # yaw 정규화 [-π, π]
        self.odom_yaw = math.atan2(
            math.sin(self.odom_yaw), math.cos(self.odom_yaw)
        )

        # /odom 발행
        self._publish_odom(msg.header.stamp)

    def _publish_odom(self, stamp):
        """Odometry 메시지 + TF 발행"""
        # 쿼터니언 계산
        cy = math.cos(self.odom_yaw * 0.5)
        sy = math.sin(self.odom_yaw * 0.5)

        # nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.twist.twist.linear.x = self.odom_vx
        odom.twist.twist.linear.y = self.odom_vy
        odom.twist.twist.angular.z = self.odom_omega

        # 공분산 (엔코더 기반 — ground truth보다 불확실)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # yaw
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)



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
