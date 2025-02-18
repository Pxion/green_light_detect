// Copyright (c) 2025 Pxion
// Licensed under the Apache-2.0 License.

#ifndef GREEN_LIGHT_DETECT_HPP_
#define GREEN_LIGHT_DETECT_HPP_

#include <opencv2/core.hpp>
#include <vector>
#include <opencv2/imgproc.hpp>

namespace green_light_detect
{
    // 检测器类，用于检测图像中的绿色光点中心
    class Detector
    {
    public:
        // 构造函数
        Detector();

        // 主要检测函数，接收输入图像，返回绿光中心坐标
        std::vector<cv::Point> detect(const cv::Mat& input_image,cv::Mat& output_image);

    private:
        // 图像预处理函数：将图像转换到HSV空间，提取绿色区域，二值化，形态学操作
        cv::Mat preprocessImage(const cv::Mat& input_image);

        // 查找绿光中心坐标函数：查找轮廓并计算符合条件的轮廓的中心坐标
        std::vector<cv::Point> findGreenLightCenters(const cv::Mat& processed_image,cv::Mat& output_image);

        // 均值滤波函数：对检测到的坐标进行均值滤波处理
        std::vector<cv::Point> meanFilter(const std::vector<cv::Point>& coords, int window_size = 5);
    };
}  // namespace green_light_detect

#endif  // GREEN_LIGHT_DETECT_HPP_
