#!/usr/bin/env python3
"""
IsaacSim ↔ ApexNAV topic bridge node.

Converts IsaacSim topics to ApexNAV expected format:
  /odom              → /habitat/odom        (frame_id: odom → world)
  /odom              → /habitat/sensor_pose (Habitat forward transform + camera height)
  /zed_mini/rgb      → /habitat/camera_rgb  (frame_id → world)
  /zed_mini/depth_image → /habitat/camera_depth (normalize meters → [0, 1])

QoS for publishers: BEST_EFFORT, KEEP_LAST, depth=10
QoS for subscribers: /odom RELIABLE, /zed_mini/* RELIABLE VOLATILE
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion, quaternion_from_euler


# QoS profiles
PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

SUB_ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

SUB_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class IsaacSimApexNavBridge(Node):
    def __init__(self):
        super().__init__("isaacsim_apexnav_bridge")

        # Parameters
        self.declare_parameter("camera_height", 0.88)
        self.declare_parameter("max_depth", 5.0)

        self.camera_height = self.get_parameter("camera_height").value
        self.max_depth = self.get_parameter("max_depth").value

        self.bridge = CvBridge()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/habitat/odom", PUB_QOS)
        self.sensor_pose_pub = self.create_publisher(
            Odometry, "/habitat/sensor_pose", PUB_QOS
        )
        self.rgb_pub = self.create_publisher(Image, "/habitat/camera_rgb", PUB_QOS)
        self.depth_pub = self.create_publisher(
            Image, "/habitat/camera_depth", PUB_QOS
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, SUB_ODOM_QOS
        )
        self.rgb_sub = self.create_subscription(
            Image, "/zed_mini/rgb", self.rgb_callback, SUB_IMAGE_QOS
        )
        self.depth_sub = self.create_subscription(
            Image, "/zed_mini/depth", self.depth_callback, SUB_IMAGE_QOS
        )

        self.get_logger().info(
            f"IsaacSim↔ApexNAV bridge started "
            f"(camera_height={self.camera_height}m, max_depth={self.max_depth}m)"
        )

    # ------------------------------------------------------------------
    # /odom → /habitat/odom
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        # --- /habitat/odom: same data, frame_id changed to "world" ---
        odom_out = Odometry()
        odom_out.header = msg.header
        odom_out.header.frame_id = "world"
        odom_out.child_frame_id = msg.child_frame_id
        odom_out.pose = msg.pose
        odom_out.twist = msg.twist
        self.odom_pub.publish(odom_out)

        # --- /habitat/sensor_pose: Habitat forward transform ---
        self.sensor_pose_pub.publish(
            self._make_sensor_pose(msg)
        )

    def _make_sensor_pose(self, odom_msg: Odometry) -> Odometry:
        """
        Apply the Habitat publisher forward transform (habitat_publisher.py:62-75).

        IsaacSim ROS odom:
          pos = (x_ros, y_ros, z_ros≈0), yaw = yaw_ros

        Habitat GPS mapping:
          gps[0] = -y_ros,  gps[1] = z_ros ≈ 0,  gps[2] = -x_ros
          compass = yaw_ros,  pitch = 0

        habitat_publisher.publish_camera_odom forward transform:
          position.x = -gps[2]           =  x_ros
          position.y = -gps[0]           =  y_ros
          position.z = gps[1] + height   =  camera_height   (z_ros ≈ 0)
          orientation = quaternion_from_euler(pitch + pi/2, pi, compass + pi/2)
                      = quaternion_from_euler(pi/2, pi, yaw_ros + pi/2)

        Inverse (real_world_test_habitat.py:35-51) recovers gps and compass correctly.
        """
        pos = odom_msg.pose.pose.position
        orn = odom_msg.pose.pose.orientation

        # Extract yaw from odom orientation
        _, _, yaw_ros = euler_from_quaternion(
            [orn.x, orn.y, orn.z, orn.w]
        )

        # Forward transform
        q = quaternion_from_euler(math.pi / 2.0, math.pi, yaw_ros + math.pi / 2.0)

        sensor_pose = Odometry()
        sensor_pose.header = odom_msg.header
        sensor_pose.header.frame_id = "world"
        sensor_pose.child_frame_id = "base_link"
        sensor_pose.pose.pose = Pose(
            position=Point(
                x=float(pos.x),
                y=float(pos.y),
                z=float(self.camera_height),
            ),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )
        return sensor_pose

    # ------------------------------------------------------------------
    # /zed_mini/rgb → /habitat/camera_rgb
    # ------------------------------------------------------------------
    def rgb_callback(self, msg: Image) -> None:
        out = Image()
        out.header = msg.header
        out.header.frame_id = "world"
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data
        self.rgb_pub.publish(out)

    # ------------------------------------------------------------------
    # /zed_mini/depth_image → /habitat/camera_depth (normalized [0, 1])
    # ------------------------------------------------------------------
    def depth_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth conversion failed: {e}")
            return

        # Handle NaN / Inf
        cv_image = np.nan_to_num(
            cv_image, nan=0.0, posinf=self.max_depth, neginf=0.0
        )

        # Normalize meters → [0, 1]
        cv_image = np.clip(cv_image / self.max_depth, 0.0, 1.0).astype(np.float32)

        try:
            depth_out = self.bridge.cv2_to_imgmsg(cv_image, encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth encoding failed: {e}")
            return

        depth_out.header = msg.header
        depth_out.header.frame_id = "world"
        self.depth_pub.publish(depth_out)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimApexNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
