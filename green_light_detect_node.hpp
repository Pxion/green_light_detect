// Copyright (c) 2025 Pxion
// Licensed under the Apache-2.0 License.

#ifndef GREEN_LIGHT_DETECTOR_NODE_HPP_
#define GREEN_LIGHT_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include </home/pxion/ros_ws/src/green_light_detect/include/green_light_detect/green_light_dectet.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class GreenLightDetectorNode : public rclcpp::Node
{
public:
    // 构造函数：初始化节点并设置订阅和发布
    GreenLightDetectorNode();

private:
    // 图像回调函数，用于接收图像并处理
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // 订阅者：订阅图像话题
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // 发布者：发布检测到的绿色光点中心坐标
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marked_image_pub_;
};

#endif  // GREEN_LIGHT_DETECTOR_NODE_HPP_
