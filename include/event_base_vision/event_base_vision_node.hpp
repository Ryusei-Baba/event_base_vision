#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "event_base_vision/visibility_control.h"

namespace event_base_vision {

class EventBaseVisionNode : public rclcpp::Node {
public:
    EVENT_BASE_VISION_PUBLIC
    explicit EventBaseVisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishEventImage(const cv::Mat &event_image, const sensor_msgs::msg::Image::SharedPtr &msg);

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr event_image_pub_;
    cv::Mat previous_image_; // 前回の画像
    bool use_event_base_vision_; // イベントベースビジョンを使用するかのフラグ
};

} // namespace event_base_vision
