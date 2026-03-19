#!/usr/bin/env python3
"""IsaacSim ↔ Nav2 TF Bridge

Static TF만 발행: odom → World (identity)
- IsaacSim TF: World → world → base_link
- 이 브릿지: odom → World
- 결과 체인: odom → World → world → base_link

/odom 메시지는 swerve_controller.py가 FK 기반으로 발행합니다.

사전 조건:
- IsaacSim Play 상태
- Step 4 TF publisher 동작 중
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.get_logger().info(
            'Nav2 Bridge started\n'
            '  - Static TF: odom → World (identity)'
        )

    def _publish_static_tf(self):
        """odom → World 정적 변환 발행 (동일 좌표계)"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'World'
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)


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
