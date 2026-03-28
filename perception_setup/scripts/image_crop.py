#!/usr/bin/env python3

import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image


class ImageCrop(Node):
    def __init__(self):
        super().__init__("image_crop")

        self.declare_parameter("image_topic", Parameter.Type.STRING)
        self.declare_parameter("camera_info_topic", Parameter.Type.STRING)
        self.declare_parameter("output_image_topic", Parameter.Type.STRING)
        self.declare_parameter("output_camera_info_topic", Parameter.Type.STRING)
        self.declare_parameter("crop.x_offset", Parameter.Type.INTEGER)
        self.declare_parameter("crop.y_offset", Parameter.Type.INTEGER)
        self.declare_parameter("crop.width", Parameter.Type.INTEGER)
        self.declare_parameter("crop.height", Parameter.Type.INTEGER)
        self.declare_parameter("enable_crop", Parameter.Type.BOOL)

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        out_image_topic = self.get_parameter("output_image_topic").value
        out_info_topic = self.get_parameter("output_camera_info_topic").value

        self.x = self.get_parameter("crop.x_offset").value
        self.y = self.get_parameter("crop.y_offset").value
        self.w = self.get_parameter("crop.width").value
        self.h = self.get_parameter("crop.height").value
        self.enable_crop = self.get_parameter("enable_crop").value

        self.bridge = CvBridge()

        image_sub = Subscriber(self, Image, image_topic)
        info_sub = Subscriber(self, CameraInfo, info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [image_sub, info_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.callback)

        self.image_pub = self.create_publisher(Image, out_image_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, out_info_topic, 10)

        if self.enable_crop:
            self.get_logger().info(
                f"image_crop: {image_topic} -> {out_image_topic} "
                f"[x={self.x}, y={self.y}, w={self.w or 'full'}, h={self.h or 'full'}]"
            )
        else:
            self.get_logger().info(
                f"image_crop: passthrough {image_topic} -> {out_image_topic}"
            )

    def callback(self, image_msg: Image, info_msg: CameraInfo):
        if not self.enable_crop:
            self.image_pub.publish(image_msg)
            self.info_pub.publish(info_msg)
            return

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

        img_h, img_w = cv_image.shape[:2]

        x = self.x
        y = self.y
        w = self.w if self.w > 0 else img_w - x
        h = self.h if self.h > 0 else img_h - y

        cropped = cv_image[y : y + h, x : x + w]

        # Republish cropped image
        out_image = self.bridge.cv2_to_imgmsg(cropped, encoding=image_msg.encoding)
        out_image.header = image_msg.header
        self.image_pub.publish(out_image)

        # Update camera info: shift principal point by crop offset, update size
        out_info = CameraInfo()
        out_info.header = info_msg.header
        out_info.width = w
        out_info.height = h
        out_info.distortion_model = info_msg.distortion_model
        out_info.d = info_msg.d

        K = list(info_msg.k)
        K[2] -= x  # cx
        K[5] -= y  # cy
        out_info.k = K

        out_info.r = info_msg.r

        P = list(info_msg.p)
        P[2] -= x  # cx
        P[6] -= y  # cy
        out_info.p = P

        out_info.roi.x_offset = x
        out_info.roi.y_offset = y
        out_info.roi.width = w
        out_info.roi.height = h
        out_info.roi.do_rectify = False

        self.info_pub.publish(out_info)


def main():
    rclpy.init()
    node = ImageCrop()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
