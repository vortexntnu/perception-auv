#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('gripper_camera_info_publisher')

        # --- Example camera intrinsics (change these for your camera) ---
        self.width = 720
        self.height = 540
        fx = 500.0
        fy = 500.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        # Prepare static camera info message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = 'gripper_camera'
        self.camera_info_msg.width = self.width
        self.camera_info_msg.height = self.height

        # Intrinsic matrix K
        self.camera_info_msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P
        self.camera_info_msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Rectification matrix (identity)
        self.camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Distortion (none)
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.distortion_model = 'plumb_bob'

        # Publisher and subscriber
        self.publisher_ = self.create_publisher(CameraInfo, '/gripper_camera/camera_info', 10)
        self.subscription = self.create_subscription(
            Image,
            '/gripper_camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Subscribing to /gripper_camera/image_raw and publishing synchronized /gripper_camera/camera_info')

    def image_callback(self, msg: Image):
        """Called whenever a new image arrives"""
        self.camera_info_msg.header.stamp = msg.header.stamp
        self.publisher_.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
