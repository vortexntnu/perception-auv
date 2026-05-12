#!/usr/bin/env python3
"""Depth image to point cloud converter for ROS2 Humble."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header


class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_node')

        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw_cropped')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info_cropped')
        self.declare_parameter('pointcloud_topic', '/camera/camera/depth/points')
        self.declare_parameter('depth_scale', 0.001)  # mm -> m for 16UC1
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('max_depth', 100000.0)
        self.declare_parameter('stride', 1)  # downsample factor (1 = no downsample)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.pc_topic = self.get_parameter('pointcloud_topic').value
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.stride = int(self.get_parameter('stride').value)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.info_frame = None
        self._u_grid = None
        self._v_grid = None
        self._grid_shape = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, sensor_qos)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, sensor_qos)

        self.pc_pub = self.create_publisher(PointCloud2, self.pc_topic, sensor_qos)

        self.get_logger().info(
            f"depth_to_pointcloud: sub depth='{self.depth_topic}', info='{self.info_topic}', "
            f"pub points='{self.pc_topic}' (depth_scale={self.depth_scale}, stride={self.stride})")

    def info_callback(self, msg: CameraInfo):
        k = msg.k
        if len(k) >= 9 and k[0] > 0.0 and k[4] > 0.0:
            self.fx = k[0]
            self.fy = k[4]
            self.cx = k[2]
            self.cy = k[5]
            self.info_frame = msg.header.frame_id

    def _ensure_grid(self, height: int, width: int):
        if self._grid_shape == (height, width):
            return
        us = np.arange(0, width, self.stride, dtype=np.float32)
        vs = np.arange(0, height, self.stride, dtype=np.float32)
        self._u_grid, self._v_grid = np.meshgrid(us, vs)
        self._grid_shape = (height, width)

    def depth_callback(self, msg: Image):
        if self.fx is None:
            self.get_logger().warn("Waiting for CameraInfo...", throttle_duration_sec=2.0)
            return

        if msg.encoding == '16UC1':
            dtype = np.uint16
            scale = self.depth_scale
        elif msg.encoding == '32FC1':
            dtype = np.float32
            scale = 1.0
        else:
            self.get_logger().warn(
                f"Unsupported depth encoding '{msg.encoding}'", throttle_duration_sec=5.0)
            return

        try:
            depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        except ValueError as e:
            self.get_logger().error(f"Bad depth buffer shape: {e}")
            return

        if self.stride > 1:
            depth = depth[::self.stride, ::self.stride]

        self._ensure_grid(msg.height, msg.width)

        z = depth.astype(np.float32) * scale
        valid = np.isfinite(z) & (z >= self.min_depth) & (z <= self.max_depth)

        if not np.any(valid):
            header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id or self.info_frame)
            self.pc_pub.publish(self._empty_cloud(header))
            return

        z_v = z[valid]
        u_v = self._u_grid[valid]
        v_v = self._v_grid[valid]

        x = (u_v - self.cx) * z_v / self.fx
        y = (v_v - self.cy) * z_v / self.fy

        points = np.stack([x, y, z_v], axis=-1).astype(np.float32)

        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id or self.info_frame)
        cloud = self._make_xyz_cloud(header, points)
        self.pc_pub.publish(cloud)

    @staticmethod
    def _make_xyz_cloud(header: Header, points: np.ndarray) -> PointCloud2:
        n = points.shape[0]
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * n
        msg.data = points.tobytes()
        return msg

    @staticmethod
    def _empty_cloud(header: Header) -> PointCloud2:
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = 0
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 0
        msg.data = b''
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
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
