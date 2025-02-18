// Copyright (c) 2025 Pxion
// Licensed under the Apache-2.0 License.

#include </home/pxion/ros_ws/src/green_light_detect/include/green_light_detect/green_light_dectet.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>

namespace green_light_detect
{
// 构造函数实现
Detector::Detector() {}

// 主要检测函数，接收输入图像，返回绿光中心坐标，并输出标注图像
std::vector<cv::Point> Detector::detect(const cv::Mat& input_image, cv::Mat& output_image)
{
    // 克隆原始图像用于绘制标注
    output_image = input_image.clone();
    
    // 图像预处理（生成二值化图像）
    cv::Mat processed_image = preprocessImage(input_image);
    
    // 查找绿光中心坐标并在output_image上绘制
    std::vector<cv::Point> centers = findGreenLightCenters(processed_image, output_image);
    
    // 对中心坐标进行均值滤波
    std::vector<cv::Point> filtered_centers = meanFilter(centers);
    
    // 在图像上绘制滤波后的中心点（蓝色）
    for (const auto& center : filtered_centers) {
        cv::circle(output_image, center, 5, cv::Scalar(255, 0, 0), -1); // 蓝色实心圆点
    }
    
    return filtered_centers;
}

// 图像预处理函数（保持不变）
cv::Mat Detector::preprocessImage(const cv::Mat& input_image)
{
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Scalar lower_green(35, 40, 40);
    cv::Scalar upper_green(85, 255, 255);

    cv::Mat mask;
    cv::inRange(hsv_image, lower_green, upper_green, mask);

    cv::Mat thresholded;
    cv::threshold(mask, thresholded, 0, 255, cv::THRESH_BINARY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat processed_image;
    
    cv::morphologyEx(thresholded, processed_image, cv::MORPH_ERODE, kernel);
    cv::morphologyEx(processed_image, processed_image, cv::MORPH_DILATE, kernel);

    return processed_image;
}

// 查找绿光中心坐标并绘制边界框
std::vector<cv::Point> Detector::findGreenLightCenters(
    const cv::Mat& processed_image, 
    cv::Mat& output_image  // 传入需要绘制的图像
)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> centers;
    
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        double perimeter = cv::arcLength(contours[i], true);

        if (area > 300 && perimeter > 60) {  
            cv::Moments M = cv::moments(contours[i]);
            if (M.m00 != 0) {
                int cX = static_cast<int>(M.m10 / M.m00);
                int cY = static_cast<int>(M.m01 / M.m00);
                centers.push_back(cv::Point(cX, cY));

                // --- 绘制绿色边界框 ---
                cv::Rect rect = cv::boundingRect(contours[i]);
                cv::rectangle(output_image, rect, cv::Scalar(0, 255, 0), 2); // 绿色矩形框

                // --- 绘制原始中心点（红色）---
                cv::circle(output_image, cv::Point(cX, cY), 3, cv::Scalar(0, 0, 255), -1);
            }
        }
    }

    return centers;
}

// 均值滤波函数（保持不变）
std::vector<cv::Point> Detector::meanFilter(const std::vector<cv::Point>& coords, int window_size)
{
    std::vector<cv::Point> filtered_coords;
    
    for (size_t i = 0; i < coords.size(); i++) {
        int start = std::max(0, static_cast<int>(i) - window_size);
        int end = std::min(static_cast<int>(coords.size()), static_cast<int>(i) + window_size + 1);

        int sum_x = 0, sum_y = 0;
        for (int j = start; j < end; j++) {
            sum_x += coords[j].x;
            sum_y += coords[j].y;
        }

        int avg_x = sum_x / (end - start);
        int avg_y = sum_y / (end - start);

        filtered_coords.push_back(cv::Point(avg_x, avg_y));
    }

    return filtered_coords;
}
}  // namespace green_light_detect