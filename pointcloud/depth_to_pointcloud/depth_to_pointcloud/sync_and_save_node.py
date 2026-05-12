#!/usr/bin/env python3
"""Subscribe to cropped depth + color, sync them, and save every Nth pair to disk."""

import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
import message_filters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class SyncAndSaveNode(Node):
    def __init__(self):
        super().__init__('sync_and_save_node')

        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw_cropped')
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'output_dir',
            '/home/kluge7/workspaces/isaac_ros-dev/src/pointcloud/data')
        self.declare_parameter('save_every_nth', 5)
        self.declare_parameter('sync_policy', 'approximate')  # 'exact' or 'approximate'
        self.declare_parameter('slop_seconds', 0.05)
        self.declare_parameter('queue_size', 30)
        # Colormap scaling range in raw 16UC1 depth units (millimetres).
        # Min ~0 mm clips near-zero noise; max ~1125 mm (~1.1 m) covers the
        # expected close-range operating distance for valve interaction.
        self.declare_parameter('depth_colormap_value_min', 0.1)
        self.declare_parameter('depth_colormap_value_max', 1125.5)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.color_topic = self.get_parameter('color_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.save_every_nth = int(self.get_parameter('save_every_nth').value)
        self.sync_policy = self.get_parameter('sync_policy').value
        self.slop = float(self.get_parameter('slop_seconds').value)
        self.queue_size = int(self.get_parameter('queue_size').value)
        self.dmin = float(self.get_parameter('depth_colormap_value_min').value)
        self.dmax = float(self.get_parameter('depth_colormap_value_max').value)

        os.makedirs(os.path.join(self.output_dir, 'color'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'depth'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'depth_raw'), exist_ok=True)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.queue_size,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=sensor_qos)
        self.color_sub = message_filters.Subscriber(
            self, Image, self.color_topic, qos_profile=sensor_qos)

        if self.sync_policy == 'exact':
            self.sync = message_filters.TimeSynchronizer(
                [self.depth_sub, self.color_sub], self.queue_size)
        else:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.depth_sub, self.color_sub], self.queue_size, self.slop)
        self.sync.registerCallback(self.synced_cb)

        self.pair_count = 0
        self.saved_count = 0

        self.get_logger().info(
            f"sync_and_save: depth='{self.depth_topic}', color='{self.color_topic}', "
            f"sync={self.sync_policy} (slop={self.slop}s), "
            f"every {self.save_every_nth}th pair -> {self.output_dir}")

    def synced_cb(self, depth_msg: Image, color_msg: Image):
        self.pair_count += 1
        if (self.pair_count - 1) % self.save_every_nth != 0:
            return

        try:
            color_img = self._decode_color(color_msg)
            depth_img = self._decode_depth(depth_msg)
        except ValueError as e:
            self.get_logger().error(f"Decode failed: {e}")
            return

        idx = self.saved_count  # same index for both files in this pair
        name = f"frame_{idx:06d}"
        color_path = os.path.join(self.output_dir, 'color', f"{name}.png")
        depth_colored_path = os.path.join(self.output_dir, 'depth', f"{name}.png")
        depth_raw_path = os.path.join(self.output_dir, 'depth_raw', f"{name}.png")

        depth_colored = self._colorize_depth(depth_img)

        cv2.imwrite(color_path, color_img)
        cv2.imwrite(depth_colored_path, depth_colored)
        cv2.imwrite(depth_raw_path, depth_img)  # 16-bit single-channel PNG
        self.saved_count += 1

        dt_ns = abs(
            (depth_msg.header.stamp.sec - color_msg.header.stamp.sec) * 1_000_000_000
            + (depth_msg.header.stamp.nanosec - color_msg.header.stamp.nanosec))
        self.get_logger().info(
            f"Saved pair #{self.saved_count} ({name}) "
            f"depth={depth_img.shape}{depth_img.dtype} color={color_img.shape}, "
            f"dt={dt_ns/1e6:.2f}ms",
            throttle_duration_sec=0.0)

    @staticmethod
    def _decode_color(msg: Image) -> np.ndarray:
        if msg.encoding == 'bgr8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'mono8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        raise ValueError(f"Unsupported color encoding '{msg.encoding}'")

    def _colorize_depth(self, depth: np.ndarray) -> np.ndarray:
        invalid = depth == 0  # 16UC1 convention: 0 = no measurement
        d = depth.astype(np.float32)
        span = max(self.dmax - self.dmin, 1e-6)
        norm = np.clip((d - self.dmin) / span, 0.0, 1.0)
        gray = (norm * 255.0).astype(np.uint8)
        colored = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
        colored[invalid] = 0  # keep "no measurement" pixels black
        return colored

    @staticmethod
    def _decode_depth(msg: Image) -> np.ndarray:
        if msg.encoding == '16UC1':
            return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        if msg.encoding == '32FC1':
            f = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            return np.nan_to_num(f, nan=0.0).astype(np.uint16)
        raise ValueError(f"Unsupported depth encoding '{msg.encoding}'")


def main(args=None):
    rclpy.init(args=args)
    node = SyncAndSaveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
