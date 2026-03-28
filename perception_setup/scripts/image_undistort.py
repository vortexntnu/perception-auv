#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class ImageUndistort(Node):
    def __init__(self):
        super().__init__("image_undistort")

        self.declare_parameter("image_topic", Parameter.Type.STRING)
        self.declare_parameter("camera_info_topic", Parameter.Type.STRING)
        self.declare_parameter("raw_camera_info_topic", Parameter.Type.STRING)
        self.declare_parameter("output_image_topic", Parameter.Type.STRING)
        self.declare_parameter("output_camera_info_topic", Parameter.Type.STRING)
        self.declare_parameter("enable_undistort", Parameter.Type.BOOL)

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        raw_info_topic = self.get_parameter("raw_camera_info_topic").value
        out_topic = self.get_parameter("output_image_topic").value
        out_info_topic = self.get_parameter("output_camera_info_topic").value
        enable_undistort = self.get_parameter("enable_undistort").value

        info_qos = QoSProfile(depth=1)
        info_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        info_qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(Image, out_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, out_info_topic, info_qos)

        if enable_undistort:
            self.bridge = CvBridge()
            self.map1 = None
            self.map2 = None
            self.rectified_info = None

            # Camera info only needs to be received once — use transient local so we
            # also catch messages published before this node started (latched).
            self.create_subscription(CameraInfo, info_topic, self.info_callback, info_qos)
            self.create_subscription(Image, image_topic, self.image_callback, 10)
            self.get_logger().info(
                f"image_undistort: {image_topic} -> {out_topic}"
            )
        else:
            self.create_subscription(Image, image_topic, self.relay_image, 10)
            self.create_subscription(CameraInfo, raw_info_topic, self.relay_camera_info, info_qos)
            self.get_logger().info(
                f"image_undistort: passthrough {image_topic} -> {out_topic}"
            )

    def info_callback(self, msg: CameraInfo):
        if self.map1 is not None:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        h, w = msg.height, msg.width
        new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (w, h), cv2.CV_16SC2
        )
        # Build the rectified camera_info (zero distortion, updated K and P)
        self.rectified_info = CameraInfo()
        self.rectified_info.width = w
        self.rectified_info.height = h
        self.rectified_info.distortion_model = "plumb_bob"
        self.rectified_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.rectified_info.k = new_K.flatten().tolist()
        self.rectified_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        P = [
            new_K[0, 0],
            0.0,
            new_K[0, 2],
            0.0,
            0.0,
            new_K[1, 1],
            new_K[1, 2],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        self.rectified_info.p = P
        self.get_logger().info(f"Undistortion maps initialised ({w}x{h})")

    def image_callback(self, msg: Image):
        if self.map1 is None:
            self.get_logger().warn(
                "Waiting for camera_info...", throttle_duration_sec=5.0
            )
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        undistorted = cv2.remap(cv_image, self.map1, self.map2, cv2.INTER_LINEAR)

        out_msg = self.bridge.cv2_to_imgmsg(undistorted, encoding=msg.encoding)
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        self.rectified_info.header = msg.header
        self.info_pub.publish(self.rectified_info)

    def relay_image(self, msg: Image):
        self.pub.publish(msg)

    def relay_camera_info(self, msg: CameraInfo):
        self.info_pub.publish(msg)


def main():
    rclpy.init()
    node = ImageUndistort()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
