#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from vortex_utils_ros.qos_profiles import reliable_profile, sensor_data_profile


class ImageUndistort(Node):
    def __init__(self):
        super().__init__("image_undistort")

        self.declare_parameter("image_topic", Parameter.Type.STRING)
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("camera_info_file", "")
        self.declare_parameter("raw_camera_info_topic", Parameter.Type.STRING)
        self.declare_parameter("output_image_topic", Parameter.Type.STRING)
        self.declare_parameter("output_camera_info_topic", Parameter.Type.STRING)
        self.declare_parameter("enable_undistort", Parameter.Type.BOOL)
        self.declare_parameter("image_qos", "sensor_data")

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        camera_info_file = self.get_parameter("camera_info_file").value
        raw_info_topic = self.get_parameter("raw_camera_info_topic").value
        out_topic = self.get_parameter("output_image_topic").value
        out_info_topic = self.get_parameter("output_camera_info_topic").value
        enable_undistort = self.get_parameter("enable_undistort").value
        image_qos_str = self.get_parameter("image_qos").value

        image_qos = (
            reliable_profile(10)
            if image_qos_str == "reliable"
            else sensor_data_profile(10)
        )

        self.pub = self.create_publisher(Image, out_topic, image_qos)
        self.info_pub = self.create_publisher(
            CameraInfo, out_info_topic, reliable_profile(10)
        )

        if enable_undistort:
            self.bridge = CvBridge()
            self.map1 = None
            self.map2 = None
            self.rectified_info = None

            if camera_info_file:
                # Load K and D directly from the calibration YAML — maps are
                # ready immediately, no camera_info topic needed.
                self._init_maps_from_file(camera_info_file)
            else:
                # Camera info only needs to be received once — use transient
                # local so we also catch messages published before this node
                # started (latched).
                self.create_subscription(
                    CameraInfo, info_topic, self.info_callback, reliable_profile(1)
                )

            self.create_subscription(
                Image, image_topic, self.image_callback, sensor_data_profile(10)
            )
            self.get_logger().info(f"image_undistort: {image_topic} -> {out_topic}")
        else:
            self.create_subscription(
                Image, image_topic, self.relay_image, sensor_data_profile(10)
            )
            self.create_subscription(
                CameraInfo, raw_info_topic, self.relay_camera_info, reliable_profile(1)
            )
            self.get_logger().info(
                f"image_undistort: passthrough {image_topic} -> {out_topic}"
            )

    def _init_maps_from_file(self, path: str):
        with open(path) as f:
            data = yaml.safe_load(f)
        k = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        d = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)
        w = int(data["image_width"])
        h = int(data["image_height"])
        self._build_maps(k, d, w, h)
        self.get_logger().info(f"Undistortion maps initialised from file ({w}x{h})")

    def _build_maps(self, k, d, w, h):
        new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, (w, h), alpha=0)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            k, d, None, new_k, (w, h), cv2.CV_16SC2
        )
        self.rectified_info = CameraInfo()
        self.rectified_info.width = w
        self.rectified_info.height = h
        self.rectified_info.distortion_model = "plumb_bob"
        self.rectified_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.rectified_info.k = new_k.flatten().tolist()
        self.rectified_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.rectified_info.p = [
            new_k[0, 0], 0.0, new_k[0, 2], 0.0,
            0.0, new_k[1, 1], new_k[1, 2], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

    def info_callback(self, msg: CameraInfo):
        if self.map1 is not None:
            return
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64)
        h, w = msg.height, msg.width
        self._build_maps(k, d, w, h)
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
