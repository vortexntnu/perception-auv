#!/usr/bin/env python3
"""Crop a depth image to an ROI and republish with adjusted CameraInfo intrinsics."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class DepthCropperNode(Node):
    def __init__(self):
        super().__init__('depth_cropper_node')

        self.declare_parameter('depth_topic_in', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('info_topic_in', '/camera/camera/depth/camera_info')
        self.declare_parameter('depth_topic_out', '/camera/camera/depth/image_rect_raw_cropped')
        self.declare_parameter('info_topic_out', '/camera/camera/depth/camera_info_cropped')
        self.declare_parameter('x_offset', 260)
        self.declare_parameter('y_offset', 190)
        self.declare_parameter('width', 485)
        self.declare_parameter('height', 245)
        self.declare_parameter('max_depth_value', 2500.0)  # raw depth units (mm for 16UC1)

        self.depth_in = self.get_parameter('depth_topic_in').value
        self.info_in = self.get_parameter('info_topic_in').value
        self.depth_out = self.get_parameter('depth_topic_out').value
        self.info_out = self.get_parameter('info_topic_out').value
        self.x_off = int(self.get_parameter('x_offset').value)
        self.y_off = int(self.get_parameter('y_offset').value)
        self.crop_w = int(self.get_parameter('width').value)
        self.crop_h = int(self.get_parameter('height').value)
        self.max_depth_value = float(self.get_parameter('max_depth_value').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.depth_sub = self.create_subscription(
            Image, self.depth_in, self.depth_callback, sensor_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_in, self.info_callback, sensor_qos)

        self.depth_pub = self.create_publisher(Image, self.depth_out, sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, self.info_out, sensor_qos)

        self.get_logger().info(
            f"depth_cropper: ROI x={self.x_off} y={self.y_off} "
            f"w={self.crop_w} h={self.crop_h}, max_depth_value={self.max_depth_value}; "
            f"in='{self.depth_in}' / '{self.info_in}' -> "
            f"out='{self.depth_out}' / '{self.info_out}'")

    def _bounded_roi(self, full_w: int, full_h: int):
        x = max(0, min(self.x_off, full_w))
        y = max(0, min(self.y_off, full_h))
        w = max(0, min(self.crop_w, full_w - x))
        h = max(0, min(self.crop_h, full_h - y))
        return x, y, w, h

    def depth_callback(self, msg: Image):
        if msg.encoding not in ('16UC1', '32FC1'):
            self.get_logger().warn(
                f"Unsupported depth encoding '{msg.encoding}'", throttle_duration_sec=5.0)
            return

        dtype = np.uint16 if msg.encoding == '16UC1' else np.float32
        bytes_per_px = 2 if msg.encoding == '16UC1' else 4

        try:
            img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        except ValueError as e:
            self.get_logger().error(f"Bad depth buffer shape: {e}")
            return

        x, y, w, h = self._bounded_roi(msg.width, msg.height)
        if w == 0 or h == 0:
            self.get_logger().warn("ROI is empty after bounding to image size",
                                   throttle_duration_sec=5.0)
            return

        cropped = img[y:y + h, x:x + w].copy()
        cropped[cropped > self.max_depth_value] = 0  # 0 means "no measurement"
        cropped = np.ascontiguousarray(cropped)

        out = Image()
        out.header = msg.header
        out.height = h
        out.width = w
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = w * bytes_per_px
        out.data = cropped.tobytes()
        self.depth_pub.publish(out)

    def info_callback(self, msg: CameraInfo):
        x, y, w, h = self._bounded_roi(msg.width, msg.height)
        if w == 0 or h == 0:
            return

        out = CameraInfo()
        out.header = msg.header
        out.height = h
        out.width = w
        out.distortion_model = msg.distortion_model
        out.d = list(msg.d)

        k = list(msg.k)
        if len(k) == 9:
            k[2] = k[2] - x  # cx' = cx - x_offset
            k[5] = k[5] - y  # cy' = cy - y_offset
        out.k = k

        out.r = list(msg.r)

        p = list(msg.p)
        if len(p) == 12:
            p[2] = p[2] - x   # cx' in P
            p[6] = p[6] - y   # cy' in P
        out.p = p

        out.binning_x = msg.binning_x
        out.binning_y = msg.binning_y
        out.roi.x_offset = x
        out.roi.y_offset = y
        out.roi.width = w
        out.roi.height = h
        out.roi.do_rectify = True
        self.info_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthCropperNode()
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
