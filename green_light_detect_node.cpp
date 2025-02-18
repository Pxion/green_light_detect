// Copyright (c) 2025 Pxion
// Licensed under the Apache-2.0 License.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include </home/pxion/ros_ws/src/green_light_detect/include/green_light_detect/green_light_dectet.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/qos.hpp>

class GreenLightDetectorNode : public rclcpp::Node
{
public:
    GreenLightDetectorNode() : Node("green_light_detector_node")
    {
        // 配置QoS以匹配发布者（BEST_EFFORT + KeepLast(10)）
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .best_effort()
                                      .durability_volatile();

        // 订阅图像话题
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 
            qos_profile,
            std::bind(&GreenLightDetectorNode::imageCallback, this, std::placeholders::_1)
        );

        // 发布中心坐标
        point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/green_light/center", 10);

        // 初始化标注图像发布者
        marked_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/green_light/marked_image", 
            rclcpp::SensorDataQoS()  // BEST_EFFORT策略
        );

        RCLCPP_INFO(this->get_logger(), "Green Light Detector Node Initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat marked_image;

            // 调用检测函数并生成标注图像
            green_light_detect::Detector detector;
            std::vector<cv::Point> centers = detector.detect(cv_ptr->image, marked_image);

            // 发布标注后的图像
            auto marked_msg = cv_bridge::CvImage(msg->header, "bgr8", marked_image).toImageMsg();
            marked_image_pub_->publish(*marked_msg);

            // 发布中心点坐标
            for (const auto &center : centers)
            {
                geometry_msgs::msg::Point point_msg;
                point_msg.x = center.x;
                point_msg.y = center.y;
                point_pub_->publish(point_msg);
                RCLCPP_INFO(this->get_logger(), "Detected Green Light Center: (%.2f, %.2f)", 
                            static_cast<double>(center.x), static_cast<double>(center.y));
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Processing error: %s", e.what());
        }
    }

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marked_image_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GreenLightDetectorNode>();
    
    // 使用多线程执行器提升性能
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}