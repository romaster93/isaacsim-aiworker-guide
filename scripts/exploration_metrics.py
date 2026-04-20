#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    from plan_env.msg import MultipleMasksWithConfidence
    HAS_CLOUDS_MSG = True
except ImportError:
    HAS_CLOUDS_MSG = False


class ExplorationMetrics(Node):
    def __init__(self, args):
        super().__init__('exploration_metrics')

        self.mode = args.mode
        self.target = args.target
        self.timeout = args.timeout
        self.success_radius = args.success_radius
        self.motion_epsilon = args.motion_epsilon
        self.output_dir = os.path.expanduser(args.output_dir)

        os.makedirs(self.output_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'metrics_{self.mode}_{ts}.csv')

        self.wall_start = time.time()
        self.path_length = 0.0
        self.motion_time_s = 0.0
        self.last_odom_time = None
        self.last_odom_x = None
        self.last_odom_y = None

        self.goal_x = None
        self.goal_y = None
        self.current_label = self.target
        self.done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_x = None
        self.start_y = None
        self._try_record_start()

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            best_effort_qos,
        )

        self.label_sub = self.create_subscription(
            String,
            '/detector/label',
            self._label_cb,
            10,
        )

        if HAS_CLOUDS_MSG:
            self.clouds_sub = self.create_subscription(
                MultipleMasksWithConfidence,
                '/detector/clouds_with_scores',
                self._clouds_cb,
                10,
            )
        else:
            self.get_logger().warn(
                'plan_env.msg.MultipleMasksWithConfidence not available — '
                'cloud subscribe skipped; success auto-detection disabled. '
                'Node will run until --timeout or Ctrl+C.'
            )

        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'exploration_metrics started: mode={self.mode} target={self.target} '
            f'timeout={self.timeout}s success_radius={self.success_radius}m '
            f'motion_epsilon={self.motion_epsilon}m/s output={self.csv_path}'
        )

    def _try_record_start(self):
        try:
            tf = self.tf_buffer.lookup_transform('World', 'base_link', rclpy.time.Time())
            self.start_x = tf.transform.translation.x
            self.start_y = tf.transform.translation.y
            self.get_logger().info(f'Start position: ({self.start_x:.3f}, {self.start_y:.3f})')
        except Exception:
            pass  # will retry in tick

    def _odom_cb(self, msg: Odometry):
        now = time.time()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_odom_x is not None:
            dx = x - self.last_odom_x
            dy = y - self.last_odom_y
            self.path_length += math.sqrt(dx * dx + dy * dy)

        self.last_odom_x = x
        self.last_odom_y = y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        linear_speed = math.sqrt(vx * vx + vy * vy)

        if linear_speed > self.motion_epsilon and self.last_odom_time is not None:
            self.motion_time_s += (now - self.last_odom_time)

        self.last_odom_time = now

    def _label_cb(self, msg: String):
        self.current_label = msg.data

    def _clouds_cb(self, msg):
        if self.goal_x is not None:
            return
        for mask in msg.masks:
            if not mask.points:
                continue
            cx = sum(p.x for p in mask.points) / len(mask.points)
            cy = sum(p.y for p in mask.points) / len(mask.points)
            self.goal_x = cx
            self.goal_y = cy
            self.get_logger().info(f'Goal centroid recorded: ({cx:.3f}, {cy:.3f})')
            break

    def _tick(self):
        if self.done:
            return

        if self.start_x is None:
            self._try_record_start()

        elapsed = time.time() - self.wall_start

        # check success
        if self.goal_x is not None:
            try:
                tf = self.tf_buffer.lookup_transform('World', 'base_link', rclpy.time.Time())
                rx = tf.transform.translation.x
                ry = tf.transform.translation.y
                dist = math.sqrt((rx - self.goal_x) ** 2 + (ry - self.goal_y) ** 2)
                if dist < self.success_radius:
                    self._finish(success=True, duration=elapsed)
                    return
            except Exception:
                pass

        if elapsed >= self.timeout:
            self._finish(success=False, duration=elapsed)

    def _finish(self, success: bool, duration: float):
        self.done = True

        start_x = self.start_x if self.start_x is not None else float('nan')
        start_y = self.start_y if self.start_y is not None else float('nan')
        goal_x = self.goal_x if self.goal_x is not None else float('nan')
        goal_y = self.goal_y if self.goal_y is not None else float('nan')

        if success and self.goal_x is not None and self.start_x is not None:
            optimal_length = math.sqrt((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2)
            spl = optimal_length / max(self.path_length, optimal_length) if self.path_length > 0 else 1.0
        else:
            optimal_length = float('nan')
            spl = 0.0

        idle_time_s = duration - self.motion_time_s
        mean_speed = self.path_length / self.motion_time_s if self.motion_time_s > 0 else 0.0

        row = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'mode': self.mode,
            'target_label': self.current_label,
            'success': int(success),
            'path_length': round(self.path_length, 4),
            'optimal_length': round(optimal_length, 4) if not math.isnan(optimal_length) else '',
            'spl': round(spl, 4),
            'duration_s': round(duration, 2),
            'motion_time_s': round(self.motion_time_s, 2),
            'idle_time_s': round(idle_time_s, 2),
            'mean_speed': round(mean_speed, 4),
            'start_x': round(start_x, 4) if not math.isnan(start_x) else '',
            'start_y': round(start_y, 4) if not math.isnan(start_y) else '',
            'goal_x': round(goal_x, 4) if not math.isnan(goal_x) else '',
            'goal_y': round(goal_y, 4) if not math.isnan(goal_y) else '',
        }

        fieldnames = [
            'timestamp', 'mode', 'target_label', 'success',
            'path_length', 'optimal_length', 'spl',
            'duration_s', 'motion_time_s', 'idle_time_s', 'mean_speed',
            'start_x', 'start_y', 'goal_x', 'goal_y',
        ]

        write_header = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        idle_pct = (idle_time_s / duration * 100) if duration > 0 else 0.0
        opt_str = f'{optimal_length:.1f}m' if not math.isnan(optimal_length) else 'N/A'
        print(
            f'[METRICS] mode={self.mode} target={self.current_label} success={int(success)} '
            f'path={self.path_length:.1f}m optimal={opt_str}\n'
            f'          spl={spl:.3f} duration={duration:.1f}s motion={self.motion_time_s:.1f}s '
            f'(idle={idle_time_s:.1f}s, {idle_pct:.1f}% planning) mean={mean_speed:.2f}m/s\n'
            f'          \u2192 {self.csv_path}'
        )

        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Exploration metrics recorder (SR/SPL/Time)')
    parser.add_argument('--mode', required=True, choices=['ackermann', 'swerve'],
                        help='Drive mode for CSV labeling')
    parser.add_argument('--target', default='chair',
                        help='Target object label (default: chair)')
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='Auto-terminate after this many seconds (default: 300)')
    parser.add_argument('--success-radius', type=float, default=1.0,
                        help='Distance from cloud centroid to declare success in metres (default: 1.0)')
    parser.add_argument('--motion-epsilon', type=float, default=0.05,
                        help='Odom linear velocity threshold for "moving" in m/s (default: 0.05)')
    parser.add_argument('--output-dir', default='~/ms_AIworker/results/',
                        help='Directory for CSV output (default: ~/ms_AIworker/results/)')

    # strip ROS args before argparse
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ExplorationMetrics(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        elapsed = time.time() - node.wall_start
        if not node.done:
            node._finish(success=False, duration=elapsed)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
