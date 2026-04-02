#pragma once

#include <opencv2/calib3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace perception_setup {

class ImageUndistort : public rclcpp::Node {
   public:
    explicit ImageUndistort(const rclcpp::NodeOptions& options);

   private:
    void init_maps_from_file(const std::string& path);
    void build_maps(const cv::Mat& k, const cv::Mat& d, int w, int h);

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void relay_image(const sensor_msgs::msg::Image::SharedPtr msg);
    void relay_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

    cv::Mat map1_, map2_;
    sensor_msgs::msg::CameraInfo rectified_info_;
    bool maps_ready_{false};
};

}  // namespace perception_setup
