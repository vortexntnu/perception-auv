#include "perception_setup/image_undistort.hpp"

#include <yaml-cpp/yaml.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace perception_setup {

ImageUndistort::ImageUndistort(const rclcpp::NodeOptions& options)
    : Node("image_undistort", options) {
    const auto image_topic = declare_parameter<std::string>("image_topic");
    const auto info_topic =
        declare_parameter<std::string>("camera_info_topic", "");
    const auto camera_info_file =
        declare_parameter<std::string>("camera_info_file", "");
    const auto raw_info_topic =
        declare_parameter<std::string>("raw_camera_info_topic");
    const auto out_topic = declare_parameter<std::string>("output_image_topic");
    const auto out_info_topic =
        declare_parameter<std::string>("output_camera_info_topic");
    const auto enable_undistort = declare_parameter<bool>("enable_undistort");
    const auto image_qos_str =
        declare_parameter<std::string>("image_qos", "sensor_data");

    const auto image_qos = (image_qos_str == "reliable")
                               ? rclcpp::QoS(10).reliable()
                               : rclcpp::QoS(10).best_effort();
    const auto reliable_qos = rclcpp::QoS(10).reliable();
    const auto sensor_data_qos = rclcpp::QoS(10).best_effort();

    image_pub_ =
        create_publisher<sensor_msgs::msg::Image>(out_topic, image_qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(out_info_topic,
                                                               reliable_qos);

    if (enable_undistort) {
        if (!camera_info_file.empty()) {
            init_maps_from_file(camera_info_file);
        } else {
            info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                info_topic, rclcpp::QoS(1).reliable(),
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                    info_callback(msg);
                });
        }

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic, sensor_data_qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                image_callback(msg);
            });

        RCLCPP_INFO(get_logger(), "image_undistort: %s -> %s",
                    image_topic.c_str(), out_topic.c_str());
    } else {
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic, sensor_data_qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                relay_image(msg);
            });

        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            raw_info_topic, rclcpp::QoS(1).reliable(),
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                relay_camera_info(msg);
            });

        RCLCPP_INFO(get_logger(), "image_undistort: passthrough %s -> %s",
                    image_topic.c_str(), out_topic.c_str());
    }
}

void ImageUndistort::init_maps_from_file(const std::string& path) {
    const YAML::Node data = YAML::LoadFile(path);

    const auto k_vec = data["camera_matrix"]["data"].as<std::vector<double>>();
    const auto d_vec =
        data["distortion_coefficients"]["data"].as<std::vector<double>>();
    const int w = data["image_width"].as<int>();
    const int h = data["image_height"].as<int>();

    const cv::Mat k(3, 3, CV_64F, const_cast<double*>(k_vec.data()));
    const cv::Mat d(1, static_cast<int>(d_vec.size()), CV_64F,
                    const_cast<double*>(d_vec.data()));

    build_maps(k, d, w, h);
    RCLCPP_INFO(get_logger(), "Undistortion maps initialised from file (%dx%d)",
                w, h);
}

void ImageUndistort::build_maps(const cv::Mat& k,
                                const cv::Mat& d,
                                int w,
                                int h) {
    const cv::Mat new_k = cv::getOptimalNewCameraMatrix(k, d, {w, h}, 0.0);
    cv::initUndistortRectifyMap(k, d, cv::noArray(), new_k, {w, h}, CV_16SC2,
                                map1_, map2_);

    rectified_info_ = sensor_msgs::msg::CameraInfo{};
    rectified_info_.width = static_cast<uint32_t>(w);
    rectified_info_.height = static_cast<uint32_t>(h);
    rectified_info_.distortion_model = "plumb_bob";
    rectified_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    rectified_info_.k = {new_k.at<double>(0, 0),
                         0.0,
                         new_k.at<double>(0, 2),
                         0.0,
                         new_k.at<double>(1, 1),
                         new_k.at<double>(1, 2),
                         0.0,
                         0.0,
                         1.0};
    rectified_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    rectified_info_.p = {new_k.at<double>(0, 0),
                         0.0,
                         new_k.at<double>(0, 2),
                         0.0,
                         0.0,
                         new_k.at<double>(1, 1),
                         new_k.at<double>(1, 2),
                         0.0,
                         0.0,
                         0.0,
                         1.0,
                         0.0};

    maps_ready_ = true;
}

void ImageUndistort::info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (maps_ready_)
        return;

    const cv::Mat k(3, 3, CV_64F, const_cast<double*>(msg->k.data()));
    const cv::Mat d(1, static_cast<int>(msg->d.size()), CV_64F,
                    const_cast<double*>(msg->d.data()));

    build_maps(k, d, static_cast<int>(msg->width),
               static_cast<int>(msg->height));
    RCLCPP_INFO(get_logger(), "Undistortion maps initialised (%dx%d)",
                msg->width, msg->height);

    info_sub_.reset();
}

void ImageUndistort::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!maps_ready_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for camera_info...");
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat undistorted;
    cv::remap(cv_ptr->image, undistorted, map1_, map2_, cv::INTER_LINEAR);

    auto out_msg = cv_bridge::CvImage(msg->header, msg->encoding, undistorted)
                       .toImageMsg();
    image_pub_->publish(*out_msg);

    rectified_info_.header = msg->header;
    info_pub_->publish(rectified_info_);
}

void ImageUndistort::relay_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    image_pub_->publish(*msg);
}

void ImageUndistort::relay_camera_info(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    info_pub_->publish(*msg);
}

}  // namespace perception_setup

RCLCPP_COMPONENTS_REGISTER_NODE(perception_setup::ImageUndistort)
