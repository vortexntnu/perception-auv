#!/usr/bin/env python3

import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from vortex_utils_ros.qos_profiles import reliable_profile


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("camera_info_publisher")

        self.declare_parameter("camera_info_file", Parameter.Type.STRING)
        self.declare_parameter("camera_info_topic", Parameter.Type.STRING)

        file_path = self.get_parameter("camera_info_file").value
        topic_name = self.get_parameter("camera_info_topic").value

        if not file_path:
            raise RuntimeError("Parameter 'camera_info_file' must be provided")

        if not topic_name:
            raise RuntimeError("Parameter 'camera_info_topic' must be provided")

        with open(file_path) as f:
            data = yaml.safe_load(f)

        msg = CameraInfo()

        msg.width = int(data["image_width"])
        msg.height = int(data["image_height"])
        msg.distortion_model = data["distortion_model"]

        msg.k = data["camera_matrix"]["data"]
        msg.d = data["distortion_coefficients"]["data"]
        msg.r = data["rectification_matrix"]["data"]
        msg.p = data["projection_matrix"]["data"]

        self.publisher = self.create_publisher(
            CameraInfo, topic_name, reliable_profile(1)
        )
        self.msg = msg

        self.create_timer(1.0, self.publish_camera_info)

        self.get_logger().info(f"Publishing CameraInfo on {topic_name} at 1 Hz")
        self.get_logger().info(f"Loaded calibration: {file_path}")

    def publish_camera_info(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main():
    rclpy.init()
    node = CameraInfoPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
