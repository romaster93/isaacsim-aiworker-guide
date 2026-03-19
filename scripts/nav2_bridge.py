#!/usr/bin/env python3
"""IsaacSim ↔ Nav2 Bridge

IsaacSim의 시뮬레이션 상태를 Nav2가 이해할 수 있는 형태로 변환합니다.

1. Static TF: odom → world (identity)
   - IsaacSim TF publisher가 world → base_link 발행
   - Nav2는 odom → base_link 필요
   - odom → world → base_link 체인으로 연결

2. /odom (nav_msgs/Odometry) 발행
   - TF에서 로봇 위치를 읽어서 /odom 메시지로 발행
   - Nav2의 local planner가 이 메시지를 사용

사전 조건:
- IsaacSim Play 상태
- Step 4 TF publisher 동작 중 (world → base_link TF 발행)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener


class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')

        # Static TF: odom → world (identity)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # TF listener: odom → base_link (through odom → world → base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /odom publisher
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', odom_qos)

        # 이전 상태 (속도 계산용)
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        self.prev_time = self.get_clock().now()

        # 50Hz 타이머
        self.create_timer(0.02, self._publish_odom)

        self.get_logger().info(
            'Nav2 Bridge started\n'
            '  - Static TF: odom → world (identity)\n'
            '  - Publishing: /odom (nav_msgs/Odometry) @ 50Hz'
        )

    def _publish_static_tf(self):
        """odom → world 정적 변환 발행 (동일 좌표계)"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'world'
        # identity transform (위치 0,0,0 / 회전 없음)
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def _publish_odom(self):
        """TF에서 로봇 위치를 읽어 /odom 메시지 발행"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
        except Exception:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt < 0.001:
            return

        # 현재 위치/방향
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation

        # 쿼터니언 → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 속도 계산 (수치 미분, world frame)
        vx_world = (x - self.prev_x) / dt
        vy_world = (y - self.prev_y) / dt
        vyaw = (yaw - self.prev_yaw) / dt

        # world frame → base frame 속도 변환
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_base = vx_world * cos_yaw + vy_world * sin_yaw
        vy_base = -vx_world * sin_yaw + vy_world * cos_yaw

        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw
        self.prev_time = now

        # Odometry 메시지 발행
        msg = Odometry()
        msg.header.stamp = trans.header.stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # 위치
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = q

        # 속도 (base frame 기준)
        msg.twist.twist.linear.x = vx_base
        msg.twist.twist.linear.y = vy_base
        msg.twist.twist.angular.z = vyaw

        # 공분산 (시뮬레이션이므로 작은 값 = 높은 신뢰도)
        msg.pose.covariance[0] = 0.001   # x
        msg.pose.covariance[7] = 0.001   # y
        msg.pose.covariance[35] = 0.001  # yaw
        msg.twist.covariance[0] = 0.001
        msg.twist.covariance[7] = 0.001
        msg.twist.covariance[35] = 0.001

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
