#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <limits>

class DepthToPointCloud : public rclcpp::Node
{
public:
    DepthToPointCloud() : Node("depth_to_pointcloud")
    {
        declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw");
        declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info");
        declare_parameter("pointcloud_topic", "/depth/pointcloud");
        declare_parameter("depth_scale", 0.001); // RealSense: mm -> m

        auto depth_topic       = get_parameter("depth_topic").as_string();
        auto camera_info_topic = get_parameter("camera_info_topic").as_string();
        auto pointcloud_topic  = get_parameter("pointcloud_topic").as_string();
        depth_scale_           = get_parameter("depth_scale").as_double();

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic, 10,
            std::bind(&DepthToPointCloud::depthCallback, this, std::placeholders::_1));

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&DepthToPointCloud::cameraInfoCallback, this, std::placeholders::_1));

        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, 10);

        RCLCPP_INFO(get_logger(), "depth_to_pointcloud node started");
        RCLCPP_INFO(get_logger(), "  depth:       %s", depth_topic.c_str());
        RCLCPP_INFO(get_logger(), "  camera_info: %s", camera_info_topic.c_str());
        RCLCPP_INFO(get_logger(), "  pointcloud:  %s", pointcloud_topic.c_str());
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_info_ = msg;
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_info_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for camera_info...");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        const auto & K = camera_info_->k;
        const double fx = K[0];
        const double fy = K[4];
        const double cx = K[2];
        const double cy = K[5];

        const int width  = cv_ptr->image.cols;
        const int height = cv_ptr->image.rows;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header   = msg->header;
        cloud.height   = 1;
        cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(static_cast<size_t>(width * height));

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        const float nan = std::numeric_limits<float>::quiet_NaN();

        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u, ++iter_x, ++iter_y, ++iter_z) {
                uint16_t raw = cv_ptr->image.at<uint16_t>(v, u);
                if (raw == 0) {
                    *iter_x = *iter_y = *iter_z = nan;
                    continue;
                }
                float z  = static_cast<float>(raw * depth_scale_);
                *iter_x  = static_cast<float>((u - cx) * z / fx);
                *iter_y  = static_cast<float>((v - cy) * z / fy);
                *iter_z  = z;
            }
        }

        pointcloud_pub_->publish(cloud);
    }

    double depth_scale_{0.001};
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   pointcloud_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPointCloud>());
    rclcpp::shutdown();
    return 0;
}
