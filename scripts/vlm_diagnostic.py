#!/usr/bin/env python3
"""
VLM diagnostic sniffer.

Subscribes to all relevant topics, reports:
  - publish rate per topic (Hz)
  - timestamp sync between rgb/depth/sensor_pose (ms)
  - object cloud centroid vs robot position (m)
  - latest ITM cosine score

Run: source ros2-bridge-env + source ApexNav install, then
  python3 vlm_diagnostic.py
"""

import time
from collections import defaultdict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from plan_env.msg import MultipleMasksWithConfidence


class Sniffer(Node):
    def __init__(self):
        super().__init__("vlm_diagnostic")
        self.start = time.time()
        self.counts = defaultdict(int)
        self.last_stamps = {}
        self.recent_stamps = {k: deque(maxlen=30) for k in ("rgb", "depth", "sensor_pose")}
        self.latest_odom = None
        self.last_itm = None

        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(Image, "/habitat/camera_rgb",
            lambda m: self._count("rgb", m.header.stamp), be)
        self.create_subscription(Image, "/habitat/camera_depth",
            lambda m: self._count("depth", m.header.stamp), be)
        self.create_subscription(Odometry, "/habitat/sensor_pose",
            lambda m: self._count("sensor_pose", m.header.stamp), be)
        self.create_subscription(Odometry, "/habitat/odom", self._odom_cb, be)
        self.create_subscription(Image, "/detector/detect_img",
            lambda m: self._count("detect_img", m.header.stamp), rel)
        self.create_subscription(Float64, "/blip2/cosine_score", self._itm_cb, rel)
        self.create_subscription(MultipleMasksWithConfidence,
            "/detector/clouds_with_scores", self._cloud_cb, rel)

        self.create_timer(2.0, self._report)
        self.get_logger().info("diagnostic sniffer started")

    def _count(self, name, stamp):
        self.counts[name] += 1
        self.last_stamps[name] = stamp
        if name in self.recent_stamps:
            self.recent_stamps[name].append(stamp.sec + stamp.nanosec * 1e-9)

    def _odom_cb(self, msg):
        self.counts["odom"] += 1
        self.latest_odom = msg

    def _itm_cb(self, msg):
        self.counts["itm"] += 1
        self.last_itm = msg.data

    def _cloud_cb(self, msg):
        self.counts["cloud"] += 1
        if self.latest_odom is None:
            return
        rp = self.latest_odom.pose.pose.position
        for i, (pc, score) in enumerate(zip(msg.point_clouds, msg.confidence_scores)):
            if len(pc.data) == 0:
                continue
            try:
                pts = list(pc2.read_points(pc, field_names=["x", "y", "z"],
                                            skip_nans=True))
            except Exception as e:
                self.get_logger().error(f"pc2 decode: {e}")
                continue
            if not pts:
                continue
            arr = np.array([[p[0], p[1], p[2]] for p in pts])
            c = arr.mean(axis=0)
            mn = arr.min(axis=0)
            mx = arr.max(axis=0)
            dist = np.linalg.norm(c[:2] - np.array([rp.x, rp.y]))
            self.get_logger().warn(
                f"cloud[{i}] score={score:.3f} n={len(pts)} "
                f"frame='{pc.header.frame_id}' "
                f"centroid=({c[0]:+.2f},{c[1]:+.2f},{c[2]:+.2f}) "
                f"range x=[{mn[0]:+.2f},{mx[0]:+.2f}] y=[{mn[1]:+.2f},{mx[1]:+.2f}] "
                f"robot=({rp.x:+.2f},{rp.y:+.2f}) dist_xy={dist:.2f}m"
            )

    def _report(self):
        t = time.time() - self.start
        if t < 1.0:
            return
        r = lambda k: self.counts[k] / t
        self.get_logger().info(
            f"[{t:5.1f}s] rates(Hz): "
            f"rgb={r('rgb'):5.1f} depth={r('depth'):5.1f} "
            f"sensor_pose={r('sensor_pose'):5.1f} odom={r('odom'):5.1f} | "
            f"detect_img={r('detect_img'):5.2f} itm={r('itm'):5.2f} "
            f"cloud={r('cloud'):5.2f} | "
            f"itm_last={self.last_itm}"
        )
        if all(k in self.last_stamps for k in ("rgb", "depth", "sensor_pose")):
            def _s(st):
                return st.sec + st.nanosec * 1e-9
            rgb_t = _s(self.last_stamps["rgb"])
            depth_t = _s(self.last_stamps["depth"])
            pose_t = _s(self.last_stamps["sensor_pose"])
            d_rd = abs(rgb_t - depth_t) * 1000
            d_rp = abs(rgb_t - pose_t) * 1000
            d_dp = abs(depth_t - pose_t) * 1000
            within_slop = max(d_rd, d_rp, d_dp) < 50.0
            self.get_logger().info(
                f"         sync(ms): |rgb-depth|={d_rd:5.1f} |rgb-pose|={d_rp:5.1f} "
                f"|depth-pose|={d_dp:5.1f}  within_slop(50ms)={within_slop}"
            )


def main():
    rclpy.init()
    node = Sniffer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
