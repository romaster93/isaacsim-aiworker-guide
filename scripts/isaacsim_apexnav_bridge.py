#!/usr/bin/env python3
"""
IsaacSim ↔ ApexNAV topic bridge node.

Converts IsaacSim topics to ApexNAV expected format:
  /odom              → /habitat/odom         (frame_id → World)
  TF(World→base_link)→ /habitat/camera_pose  (actual camera pose for C++ map_ros)
  /odom              → /habitat/sensor_pose  (Habitat forward transform for VLM pipeline)
  /zed_mini/rgb      → /habitat/camera_rgb   (frame_id → World)
  /zed_mini/depth    → /habitat/camera_depth (normalize meters → [0, 1])

Uses TF lookup for camera_pose to get robot's actual position in the fixed "World" frame.
"""

import math

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
from tf2_ros import Buffer, TransformListener
from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
)


# QoS profiles
PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
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

# Camera optical frame → robot base_link rotation quaternion (x, y, z, w)
# Camera: z-forward, x-right, y-down → Robot: x-forward, y-left, z-up
Q_CAM_TO_BASE = np.array([-0.5, 0.5, -0.5, 0.5])

# Camera offset from base_link (from TF: base_link → arm_base_link → head_link1 → head_link2)
CAM_OFFSET_X = 0.066
CAM_OFFSET_Y = 0.0
CAM_OFFSET_Z = 1.58

# Fixed frame for map
FIXED_FRAME = "World"


class IsaacSimApexNavBridge(Node):
    def __init__(self):
        super().__init__("isaacsim_apexnav_bridge")

        # Parameters
        self.declare_parameter("camera_height_habitat", 0.88)
        self.declare_parameter("max_depth", 5.0)

        self.camera_height_habitat = self.get_parameter("camera_height_habitat").value
        self.max_depth = self.get_parameter("max_depth").value

        self.bridge = CvBridge()

        # TF2 for looking up robot position in World frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/habitat/odom", PUB_QOS)
        self.camera_pose_pub = self.create_publisher(
            Odometry, "/habitat/camera_pose", PUB_QOS
        )
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
            f"(frame={FIXED_FRAME}, cam_z={CAM_OFFSET_Z}m, "
            f"max_depth={self.max_depth}m)"
        )

    # ------------------------------------------------------------------
    # /odom callback → publishes odom, camera_pose, sensor_pose
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        # --- /habitat/odom: use TF-based position in World frame ---
        tf_pos, tf_yaw = self._get_robot_pose_from_tf()
        if tf_pos is None:
            return

        odom_out = Odometry()
        odom_out.header.stamp = msg.header.stamp
        odom_out.header.frame_id = FIXED_FRAME
        odom_out.child_frame_id = "base_link"
        odom_out.pose.pose.position = Point(x=tf_pos[0], y=tf_pos[1], z=tf_pos[2])
        sy = math.sin(tf_yaw * 0.5)
        cy = math.cos(tf_yaw * 0.5)
        odom_out.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)
        odom_out.twist = msg.twist
        self.odom_pub.publish(odom_out)

        # --- /habitat/camera_pose: actual camera pose for C++ map_ros ---
        self.camera_pose_pub.publish(
            self._make_camera_pose(msg.header.stamp, tf_pos, tf_yaw)
        )

        # --- /habitat/sensor_pose: Habitat format for VLM pipeline ---
        self.sensor_pose_pub.publish(
            self._make_habitat_sensor_pose(msg.header.stamp, tf_pos, tf_yaw)
        )

    def _get_robot_pose_from_tf(self):
        """Look up World → base_link transform to get robot's actual position."""
        try:
            tf = self.tf_buffer.lookup_transform(
                FIXED_FRAME, "base_link", rclpy.time.Time()
            )
            pos = tf.transform.translation
            orn = tf.transform.rotation
            _, _, yaw = euler_from_quaternion([orn.x, orn.y, orn.z, orn.w])
            return (pos.x, pos.y, pos.z), yaw
        except Exception:
            return None, None

    def _make_camera_pose(self, stamp, robot_pos, yaw) -> Odometry:
        """
        Actual camera pose in World frame for C++ map_ros depth projection.

        orientation = q_yaw * Q_CAM_TO_BASE
        position = robot_pos + rotated camera offset
        """
        q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])
        q_cam_world = quaternion_multiply(q_yaw, Q_CAM_TO_BASE)

        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        cam_x = robot_pos[0] + cos_y * CAM_OFFSET_X - sin_y * CAM_OFFSET_Y
        cam_y = robot_pos[1] + sin_y * CAM_OFFSET_X + cos_y * CAM_OFFSET_Y
        cam_z = robot_pos[2] + CAM_OFFSET_Z

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = FIXED_FRAME
        out.child_frame_id = "base_link"
        out.pose.pose = Pose(
            position=Point(x=cam_x, y=cam_y, z=cam_z),
            orientation=Quaternion(
                x=q_cam_world[0], y=q_cam_world[1],
                z=q_cam_world[2], w=q_cam_world[3],
            ),
        )
        return out

    def _make_habitat_sensor_pose(self, stamp, robot_pos, yaw) -> Odometry:
        """Habitat publisher forward transform for VLM pipeline (Phase 3)."""
        q = quaternion_from_euler(math.pi / 2.0, math.pi, yaw + math.pi / 2.0)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = FIXED_FRAME
        out.child_frame_id = "base_link"
        out.pose.pose = Pose(
            position=Point(
                x=robot_pos[0],
                y=robot_pos[1],
                z=float(self.camera_height_habitat),
            ),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )
        return out

    # ------------------------------------------------------------------
    # /zed_mini/rgb → /habitat/camera_rgb
    # ------------------------------------------------------------------
    def rgb_callback(self, msg: Image) -> None:
        out = Image()
        out.header = msg.header
        out.header.frame_id = FIXED_FRAME
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data
        self.rgb_pub.publish(out)

    # ------------------------------------------------------------------
    # /zed_mini/depth → /habitat/camera_depth (normalized [0, 1])
    # ------------------------------------------------------------------
    def depth_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth conversion failed: {e}")
            return

        cv_image = np.nan_to_num(
            cv_image, nan=0.0, posinf=self.max_depth, neginf=0.0
        )
        cv_image = np.clip(cv_image / self.max_depth, 0.0, 1.0).astype(np.float32)

        try:
            depth_out = self.bridge.cv2_to_imgmsg(cv_image, encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth encoding failed: {e}")
            return

        depth_out.header = msg.header
        depth_out.header.frame_id = FIXED_FRAME
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
