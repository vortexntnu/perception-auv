#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo

from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy
from rclpy.qos import ReliabilityPolicy
from rclpy.parameter import Parameter

import yaml


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

        with open(file_path, "r") as f:
            data = yaml.safe_load(f)

        msg = CameraInfo()

        msg.width = int(data["image_width"])
        msg.height = int(data["image_height"])
        msg.distortion_model = data["distortion_model"]

        msg.k = data["camera_matrix"]["data"]
        msg.d = data["distortion_coefficients"]["data"]
        msg.r = data["rectification_matrix"]["data"]
        msg.p = data["projection_matrix"]["data"]

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(CameraInfo, topic_name, qos)

        # publish once
        self.publisher.publish(msg)

        self.get_logger().info(f"Published CameraInfo on {topic_name}")
        self.get_logger().info(f"Loaded calibration: {file_path}")


def main():
    rclpy.init()
    node = CameraInfoPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()