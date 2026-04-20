#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IsaacSim VLM real-world node.

Adapted from real_world_test_habitat.py for IsaacSim integration.
Subscribes to /habitat/* topics (published by isaacsim_apexnav_bridge.py),
runs VLM detection pipeline, and publishes object point clouds.

Run from /home/cho/ApexNav_ROS2_wrapper/real_world_test_example/:
  python /home/cho/ms_AIworker/scripts/isaacsim_realworld_node.py --config-name isaacsim_realworld
"""

import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import message_filters
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion

import hydra
from omegaconf import DictConfig

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from plan_env.msg import MultipleMasksWithConfidence

# ApexNav wrapper imports — must run from real_world_test_example/ or have parent on path
_this_dir = os.path.dirname(os.path.realpath(__file__))
_apexnav_dir = "/home/cho/ApexNav_ROS2_wrapper/real_world_test_example"
_apexnav_root = "/home/cho/ApexNav_ROS2_wrapper"
for _p in (_apexnav_dir, _apexnav_root):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from vlm.utils.get_object_utils import get_object
from vlm.utils.get_itm_message import get_itm_message_cosine
from llm.answer_reader.answer_reader import read_answer
from basic_utils.object_point_cloud_utils.object_point_cloud import get_object_point_cloud


def inverse_habitat_publisher_transform(sensor_pose_msg):
    """
    Inverse transform to recover Habitat gps and compass from ROS sensor_pose.

    The bridge (isaacsim_apexnav_bridge.py) applies the Habitat publisher forward
    transform:
      pos.x = x_ros,  pos.y = y_ros,  pos.z = camera_height
      orient = quaternion_from_euler(pi/2, pi, yaw_ros + pi/2)

    Inverse recovers:
      gps    = [-pos.y, pos.z - 0.88, -pos.x]
      compass = euler[2] + pi/2  (yaw extracted via euler_from_quaternion)
    """
    pos = sensor_pose_msg.pose.pose.position
    orn = sensor_pose_msg.pose.pose.orientation

    gps = np.array([-pos.y, pos.z - 0.88, -pos.x], dtype=np.float32)

    euler = euler_from_quaternion([orn.x, orn.y, orn.z, orn.w])
    compass_scalar = euler[2] + np.pi / 2.0
    compass = np.array([compass_scalar], dtype=np.float32)

    return gps, compass


class IsaacSimRealWorldNode(Node):
    def __init__(self, cfg):
        super().__init__("isaacsim_realworld_node")
        self.config = cfg

        self.bridge = CvBridge()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Synchronized subscribers (BEST_EFFORT — matches bridge publisher QoS)
        self.rgb_sub_ = message_filters.Subscriber(
            self, Image, "/habitat/camera_rgb", qos_profile=best_effort_qos
        )
        self.depth_sub_ = message_filters.Subscriber(
            self, Image, "/habitat/camera_depth", qos_profile=best_effort_qos
        )
        self.sensor_pose_sub_ = message_filters.Subscriber(
            self, Odometry, "/habitat/sensor_pose", qos_profile=best_effort_qos
        )

        # Async subscribers
        self.create_subscription(
            Odometry, "/habitat/odom", self.odom_callback, best_effort_qos
        )
        self.create_subscription(
            String, "/detector/label", self.label_callback, 1
        )

        # Publishers
        self.confidence_threshold_pub_ = self.create_publisher(
            Float64, "/detector/confidence_threshold", reliable_qos
        )
        self.itm_score_pub_ = self.create_publisher(
            Float64, "/blip2/cosine_score", reliable_qos
        )
        self.cld_with_score_pub_ = self.create_publisher(
            MultipleMasksWithConfidence, "/detector/clouds_with_scores", reliable_qos
        )
        self.detect_img_pub_ = self.create_publisher(
            Image, "/detector/detect_img", reliable_qos
        )

        # Detection synchronizer — slop=0.05 for IsaacSim timing tolerance
        self.sync_detect = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub_, self.depth_sub_, self.sensor_pose_sub_],
            queue_size=5,
            slop=0.05,
        )
        self.sync_detect.registerCallback(self.sync_detect_callback)

        # Value (ITM) synchronizer — same slop
        self.sync_value = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub_, self.depth_sub_, self.sensor_pose_sub_],
            queue_size=5,
            slop=0.05,
        )
        self.sync_value.registerCallback(self.sync_value_callback)

        # State
        self.robot_odom = None
        self.odom_stamp = None
        self.processing_detect = False
        self.processing_value = False

        # LLM config
        llm_cfg = self.config.llm
        self.llm_answer_path = llm_cfg.llm_answer_path
        self.llm_response_path = llm_cfg.llm_response_path
        self.llm_client = llm_cfg.llm_client

        # Label / LLM state
        self.label = None
        self.llm_answer = []
        self.room = None
        self.fusion_score = 0.0

        # Periodic confidence threshold publisher (keeps FSM out of INIT)
        self.create_timer(1.0, self.publish_confidence_threshold)

        self.get_logger().info("IsaacSimRealWorldNode initialized. Waiting for /detector/label ...")

    # ------------------------------------------------------------------
    # Synchronized detection callback
    # ------------------------------------------------------------------
    def sync_detect_callback(self, rgb_msg, depth_msg, sensor_pose_msg):
        if self.processing_detect:
            return
        self.processing_detect = True
        try:
            stamp = rgb_msg.header.stamp
            time_diff = abs(
                (stamp.sec + stamp.nanosec * 1e-9)
                - (sensor_pose_msg.header.stamp.sec + sensor_pose_msg.header.stamp.nanosec * 1e-9)
            )
            if time_diff > 0.1:
                return

            if self.label is None:
                self.get_logger().warn("detect: waiting for target label on /detector/label")
                return

            rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth_cv = np.expand_dims(depth_img.astype(np.float32), axis=-1)

            self.get_logger().info(f"detect: label={self.label}")

            detect_img, score_list, object_masks_list, label_list = get_object(
                self.label, rgb_cv, self.config.detector, self.llm_answer
            )

            gps, compass = inverse_habitat_publisher_transform(sensor_pose_msg)

            observations = {
                "depth": depth_cv,
                "gps": gps,
                "compass": compass,
            }

            obj_point_cloud_list = get_object_point_cloud(
                self.config, observations, object_masks_list, self
            )

            cld_msg = MultipleMasksWithConfidence()
            cld_msg.point_clouds = obj_point_cloud_list
            cld_msg.confidence_scores = score_list
            cld_msg.label_indices = label_list

            self.cld_with_score_pub_.publish(cld_msg)
            self.detect_img_pub_.publish(
                self.bridge.cv2_to_imgmsg(detect_img, encoding="rgb8")
            )
        except Exception as e:
            self.get_logger().error(f"detect: {e}")
        finally:
            self.processing_detect = False

    # ------------------------------------------------------------------
    # Synchronized ITM value callback
    # ------------------------------------------------------------------
    def sync_value_callback(self, rgb_msg, depth_msg, sensor_pose_msg):
        if self.processing_value:
            return
        self.processing_value = True
        try:
            stamp = rgb_msg.header.stamp
            time_diff = abs(
                (stamp.sec + stamp.nanosec * 1e-9)
                - (sensor_pose_msg.header.stamp.sec + sensor_pose_msg.header.stamp.nanosec * 1e-9)
            )
            if time_diff > 0.1:
                return

            if self.label is None:
                return

            rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            cosine = get_itm_message_cosine(rgb_cv, self.label, self.room)
            self.get_logger().info(f"value: cosine={cosine:.3f}")

            itm_msg = Float64()
            itm_msg.data = float(cosine)
            self.itm_score_pub_.publish(itm_msg)
        except Exception as e:
            self.get_logger().error(f"value: {e}")
        finally:
            self.processing_value = False

    # ------------------------------------------------------------------
    # Label callback
    # ------------------------------------------------------------------
    def label_callback(self, msg):
        try:
            new_label = str(msg.data)
            if new_label == self.label:
                return
            self.label = new_label
            self.get_logger().info(f"Received target label: {self.label}")
            try:
                self.llm_answer, self.room, self.fusion_score = read_answer(
                    self.llm_answer_path,
                    self.llm_response_path,
                    self.label,
                    self.llm_client,
                )
            except Exception:
                self.llm_answer = []
                self.room = None
                self.fusion_score = 0.0
        except Exception as e:
            self.get_logger().error(f"label_callback: {e}")

    # ------------------------------------------------------------------
    # Odom callback
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        try:
            self.robot_odom = msg
            self.odom_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"odom: {e}")

    # ------------------------------------------------------------------
    # Periodic confidence threshold publisher
    # ------------------------------------------------------------------
    def publish_confidence_threshold(self):
        msg = Float64()
        msg.data = float(self.fusion_score) if self.fusion_score > 0.0 else 0.3
        self.confidence_threshold_pub_.publish(msg)

    def run(self):
        self.get_logger().info("IsaacSimRealWorldNode running.")
        rclpy.spin(self)


@hydra.main(version_base=None, config_path="/home/cho/ApexNav_ROS2_wrapper/real_world_test_example/config", config_name="isaacsim_realworld")
def main(cfg: DictConfig):
    rclpy.init()
    try:
        node = IsaacSimRealWorldNode(cfg)
        node.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
