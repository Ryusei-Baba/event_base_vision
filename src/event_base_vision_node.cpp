#include "event_base_vision/event_base_vision_node.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace event_base_vision {

EventBaseVisionNode::EventBaseVisionNode(const rclcpp::NodeOptions &options)
    : Node("event_base_vision_node", options), use_event_base_vision_(true) {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed/zed_node/rgb/image_rect_color", 10,
        std::bind(&EventBaseVisionNode::imageCallback, this, std::placeholders::_1));

    event_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/zed/zed_node/event_base/image_raw", 10);

    RCLCPP_INFO(this->get_logger(), "EventBaseVisionNode initialized.");
}

void EventBaseVisionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received image, processing...");

    // 受信した画像をOpenCV形式に変換
    cv::Mat current_image;
    try {
        current_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 前回の画像が存在する場合のみ処理
    if (!previous_image_.empty()) {
        // 輝度差分を計算
        cv::Mat current_gray, previous_gray, diff_image;
        cv::cvtColor(current_image, current_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(previous_image_, previous_gray, cv::COLOR_BGR2GRAY);
        cv::absdiff(current_gray, previous_gray, diff_image);

        // 輝度変化をイベントベース画像としてパブリッシュ
        publishEventImage(diff_image, msg);
    }

    // 現在の画像を保存
    previous_image_ = current_image.clone();
}

void EventBaseVisionNode::publishEventImage(const cv::Mat &event_image, const sensor_msgs::msg::Image::SharedPtr &msg) {
    // OpenCV画像をROSの画像メッセージに変換
    std::shared_ptr<sensor_msgs::msg::Image> ros_event_image;
    try {
        ros_event_image = cv_bridge::CvImage(
                              msg->header,  // 元のメッセージのヘッダーを利用
                              "mono8",      // イメージ形式
                              event_image)  // 輝度差分イメージ
                              .toImageMsg();
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    event_image_pub_->publish(*ros_event_image);
    RCLCPP_INFO(this->get_logger(), "Published event-based image.");
}

} // namespace event_base_vision

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<event_base_vision::EventBaseVisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
