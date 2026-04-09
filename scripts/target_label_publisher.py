#!/usr/bin/env python3
"""
Target label publisher — user CLI to trigger ApexNAV object exploration.

Usage:
    python3 target_label_publisher.py [object_name] [--no-rotate]

    # Interactive mode (no argument): rotates 360 first, then prompts
    python3 target_label_publisher.py

    # Direct mode: rotates 360, then sends target
    python3 target_label_publisher.py chair

    # Skip initial rotation:
    python3 target_label_publisher.py --no-rotate
    python3 target_label_publisher.py chair --no-rotate

Publishes:
  /detector/label                (std_msgs/String)         — target object name
  /detector/confidence_threshold (std_msgs/Float64, 0.3)   — 1Hz, required for FSM INIT exit
  /move_base_simple/goal         (geometry_msgs/PoseStamped) — FSM trigger
  /blip2/cosine_score            (std_msgs/Float64, 0.0)   — initial ITM score

QoS: RELIABLE, depth=10
"""

import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

CONFIDENCE_THRESHOLD = 0.3
INITIAL_ITM_SCORE = 0.0
INIT_ROTATION_VEL = 0.2  # rad/s for initial 360-degree rotation


class TargetLabelPublisher(Node):
    def __init__(self):
        super().__init__("target_label_publisher")

        self.label_pub = self.create_publisher(
            String, "/detector/label", RELIABLE_QOS
        )
        self.confidence_pub = self.create_publisher(
            Float64, "/detector/confidence_threshold", RELIABLE_QOS
        )
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, "/move_base_simple/goal", goal_qos
        )
        self.itm_pub = self.create_publisher(
            Float64, "/blip2/cosine_score", RELIABLE_QOS
        )

        # /cmd_vel publisher for initial rotation
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # TF for ground-truth yaw (World → base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publish confidence threshold at 1 Hz — required for FSM to exit INIT state
        self.create_timer(1.0, self._publish_confidence)

        self.get_logger().info(
            "TargetLabelPublisher ready. "
            "Publishing /detector/confidence_threshold=0.3 at 1Hz."
        )

    def _get_yaw_from_tf(self):
        """Get ground-truth yaw from TF (World → base_link)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "World", "base_link", rclpy.time.Time()
            )
            q = tf.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return yaw
        except Exception:
            return None

    def do_initial_rotation(self) -> None:
        """Rotate 360 degrees using TF yaw to build initial SDF map."""
        print("[Init] Waiting for TF (World → base_link)...")
        yaw = None
        while yaw is None and rclpy.ok():
            yaw = self._get_yaw_from_tf()
            time.sleep(0.1)

        print(f"[Init] Starting 360-degree rotation (angular_vel={INIT_ROTATION_VEL} rad/s)")
        target = 2.0 * math.pi
        accumulated = 0.0
        prev_yaw = yaw

        twist = Twist()
        twist.angular.z = INIT_ROTATION_VEL

        while accumulated < target and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

            cur_yaw = self._get_yaw_from_tf()
            if cur_yaw is not None:
                delta = cur_yaw - prev_yaw
                if delta > math.pi:
                    delta -= 2.0 * math.pi
                elif delta < -math.pi:
                    delta += 2.0 * math.pi
                accumulated += abs(delta)
                prev_yaw = cur_yaw

            progress = min(accumulated / target * 100, 100)
            print(f"\r[Init] Rotation: {progress:.0f}%", end="", flush=True)

        # Stop rotation
        self.cmd_vel_pub.publish(Twist())
        print(f"\n[Init] Rotation complete ({math.degrees(accumulated):.0f} degrees). SDF map ready.")

    def _publish_confidence(self):
        msg = Float64()
        msg.data = CONFIDENCE_THRESHOLD
        self.confidence_pub.publish(msg)

    def send_target(self, object_name: str) -> None:
        """Publish label, ITM score, and goal trigger for the given object."""
        object_name = object_name.strip()
        if not object_name:
            self.get_logger().warn("Empty object name — skipping.")
            return

        # 1. Target label
        label_msg = String()
        label_msg.data = object_name
        self.label_pub.publish(label_msg)

        # 2. Initial ITM score
        itm_msg = Float64()
        itm_msg.data = INITIAL_ITM_SCORE
        self.itm_pub.publish(itm_msg)

        # 3. Trigger: arbitrary goal pose — just needs to arrive to kick FSM
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "World"
        goal_msg.pose.position.x = 0.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)

        self.get_logger().info(
            f"Target sent: '{object_name}' — "
            "/detector/label + /blip2/cosine_score + /move_base_simple/goal published."
        )

    def run_interactive(self):
        """Block on stdin and publish each entered object name."""
        print("Enter object name to search (Ctrl-C to quit):")
        while rclpy.ok():
            try:
                name = input("target> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if name:
                self.send_target(name)


def main(args=None):
    rclpy.init(args=args)
    node = TargetLabelPublisher()

    # Spin in background so timers fire while we block on input
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        cli_args = sys.argv[1:]
        cli_args = [a for a in cli_args if a not in ("--rotate", "--no-rotate")]

        # Give publishers a moment to connect
        time.sleep(0.5)

        # Initial 360-degree rotation to build SDF map
        if "--no-rotate" not in sys.argv:
            node.do_initial_rotation()
        else:
            print("[Init] Skipping initial rotation (--no-rotate)")

        if cli_args:
            node.send_target(" ".join(cli_args))
            print("Press Ctrl-C to stop (confidence_threshold timer keeps running).")
            spin_thread.join()
        else:
            node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
