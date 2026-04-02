#!/usr/bin/env python3

import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from vortex_utils_ros.qos_profiles import reliable_profile


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("camera_info_publisher")

        self.declare_parameter("camera_info_file", Parameter.Type.STRING)
        self.declare_parameter("camera_info_topic", Parameter.Type.STRING)
        self.declare_parameter("image_topic", Parameter.Type.STRING)

        file_path = self.get_parameter("camera_info_file").value
        topic_name = self.get_parameter("camera_info_topic").value
        image_topic = self.get_parameter("image_topic").value

        if not file_path:
            raise RuntimeError("Parameter 'camera_info_file' must be provided")

        if not topic_name:
            raise RuntimeError("Parameter 'camera_info_topic' must be provided")

        if not image_topic:
            raise RuntimeError("Parameter 'image_topic' must be provided")

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
            CameraInfo, topic_name, reliable_profile(10)
        )
        self.msg = msg

        self.create_subscription(Image, image_topic, self._image_callback, reliable_profile(1))

        self.get_logger().info(f"Publishing CameraInfo on {topic_name} synced to {image_topic}")
        self.get_logger().info(f"Loaded calibration: {file_path}")

    def _image_callback(self, img_msg: Image):
        self.msg.header.stamp = img_msg.header.stamp
        self.msg.header.frame_id = img_msg.header.frame_id
        self.publisher.publish(self.msg)


def main():
    rclpy.init()
    node = CameraInfoPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
