#include <memory>
#include <string>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

// #include "../../tools/loong_log.h"

using namespace std::chrono_literals;

namespace nav2_behavior_tree
{

class TakePhotoAction : public BT::SyncActionNode
{
public:
  TakePhotoAction(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf)
  {
    // loong_log::initLogging();
    node_ = rclcpp::Node::make_shared("take_photo_action");

    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&TakePhotoAction::imageCallback, this, std::placeholders::_1));

    std::cout << "--------------------[TakePhotoAction] Node initialized.------------------------" << std::endl;
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {
      BT::InputPort<std::string>("save_path")
    };
    return ports;
  }

  BT::NodeStatus tick() override
  {
    // setStatus(BT::NodeStatus::RUNNING);
    RCLCPP_INFO(node_->get_logger(), "[TakePhotoAction] Tick received, waiting for image...");

    // 等待图像
    auto start = node_->now();
    sensor_msgs::msg::Image::SharedPtr img = nullptr;

    while (rclcpp::ok()) {
      if (last_image_) {
        img = last_image_;
        break;
      }
      if ((node_->now() - start).seconds() > 2.0) {
        RCLCPP_ERROR(node_->get_logger(), "[TakePhotoAction] Timeout waiting for image.");
        return BT::NodeStatus::FAILURE;
      }
      rclcpp::spin_some(node_);
    }

    // 转 OpenCV
    cv::Mat mat;
    try {
      mat = cv_bridge::toCvCopy(img, "bgr8")->image;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Image conversion failed: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }

    // 读取保存路径
    std::string save_path;
    if (!getInput("save_path", save_path)) {
      RCLCPP_WARN(node_->get_logger(), "[TakePhotoAction] save_path not set, use default.");
      save_path = "/home/rx01109/loong_nav/picture";
    }

    // 创建文件名
    auto now = std::chrono::system_clock::now();
    auto t_c = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t_c));

    std::string filename = save_path + "/photo_" + buf + ".jpg";

    // 保存
    cv::imwrite(filename, mat);
    RCLCPP_INFO(node_->get_logger(), "[TakePhotoAction] Saved photo: %s", filename.c_str());

    return BT::NodeStatus::SUCCESS;
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_image_ = msg;
  }

  rclcpp::Node::SharedPtr node_;
  sensor_msgs::msg::Image::SharedPtr last_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::TakePhotoAction>(name, config);
    };

  factory.registerBuilder<nav2_behavior_tree::TakePhotoAction>("TakePhoto", builder);
}
