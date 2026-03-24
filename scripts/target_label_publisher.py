#!/usr/bin/env python3
"""
Target label publisher — user CLI to trigger ApexNAV object exploration.

Usage:
    python3 target_label_publisher.py [object_name]

    # Interactive mode (no argument):
    python3 target_label_publisher.py

    # Direct mode:
    python3 target_label_publisher.py chair

Publishes:
  /detector/label                (std_msgs/String)         — target object name
  /detector/confidence_threshold (std_msgs/Float64, 0.3)   — 1Hz, required for FSM INIT exit
  /move_base_simple/goal         (geometry_msgs/PoseStamped) — FSM trigger
  /blip2/cosine_score            (std_msgs/Float64, 0.0)   — initial ITM score

QoS: RELIABLE, depth=10
"""

import sys
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

CONFIDENCE_THRESHOLD = 0.3
INITIAL_ITM_SCORE = 0.0


class TargetLabelPublisher(Node):
    def __init__(self):
        super().__init__("target_label_publisher")

        self.label_pub = self.create_publisher(
            String, "/detector/label", RELIABLE_QOS
        )
        self.confidence_pub = self.create_publisher(
            Float64, "/detector/confidence_threshold", RELIABLE_QOS
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, "/move_base_simple/goal", RELIABLE_QOS
        )
        self.itm_pub = self.create_publisher(
            Float64, "/blip2/cosine_score", RELIABLE_QOS
        )

        # Publish confidence threshold at 1 Hz — required for FSM to exit INIT state
        self.create_timer(1.0, self._publish_confidence)

        self.get_logger().info(
            "TargetLabelPublisher ready. "
            "Publishing /detector/confidence_threshold=0.3 at 1Hz."
        )

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
        goal_msg.header.frame_id = "world"
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
        # If object name passed as CLI argument, send it once then stay alive
        # (confidence timer keeps running)
        cli_args = sys.argv[1:]
        if cli_args:
            import time
            # Give publishers a moment to connect
            time.sleep(0.5)
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
