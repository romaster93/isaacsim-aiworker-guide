#!/usr/bin/env python3
"""Dual LaserScan Merger
/laser_scan_left + /laser_scan_right → /scan

두 2D LiDAR 스캔을 받을 때마다 즉시 병합하여 /scan으로 발행합니다.
둘 다 frame_id가 base_link이므로 TF 변환 없이 직접 합칩니다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class LaserMerger(Node):
    def __init__(self):
        super().__init__('laser_merger')

        self.scan_left = None
        self.scan_right = None

        # QoS: LiDAR는 보통 Best Effort
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            LaserScan, '/laser_scan_left', self._cb_left, sensor_qos
        )
        self.create_subscription(
            LaserScan, '/laser_scan_right', self._cb_right, sensor_qos
        )

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(
            'Laser Merger started\n'
            '  - Input: /laser_scan_left + /laser_scan_right\n'
            '  - Output: /scan (merged per frame)'
        )

    def _cb_left(self, msg):
        self.scan_left = msg
        self._merge_and_publish()

    def _cb_right(self, msg):
        self.scan_right = msg
        self._merge_and_publish()

    def _merge_and_publish(self):
        """스캔이 올 때마다 최신 양쪽 데이터를 합쳐서 발행"""
        if self.scan_left is None or self.scan_right is None:
            return

        # 기준: left 스캔의 설정 사용
        ref = self.scan_left
        merged = LaserScan()
        merged.header.stamp = ref.header.stamp
        merged.header.frame_id = ref.header.frame_id  # base_link
        merged.angle_min = ref.angle_min
        merged.angle_max = ref.angle_max
        merged.angle_increment = ref.angle_increment
        merged.time_increment = ref.time_increment
        merged.scan_time = ref.scan_time
        merged.range_min = ref.range_min
        merged.range_max = ref.range_max

        # 양쪽 ranges 합치기: 각 빔에서 유효한 값 중 가까운 것 사용
        merged_ranges = []
        num_beams = len(ref.ranges)

        for i in range(num_beams):
            r_left = self.scan_left.ranges[i] if i < len(self.scan_left.ranges) else float('inf')
            r_right = self.scan_right.ranges[i] if i < len(self.scan_right.ranges) else float('inf')

            # 유효하지 않은 값 처리
            if math.isinf(r_left) or math.isnan(r_left) or r_left < ref.range_min:
                r_left = float('inf')
            if math.isinf(r_right) or math.isnan(r_right) or r_right < ref.range_min:
                r_right = float('inf')

            # 가까운 값 선택
            merged_ranges.append(min(r_left, r_right))

        merged.ranges = merged_ranges
        self.scan_pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = LaserMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
