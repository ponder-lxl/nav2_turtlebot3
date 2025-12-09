// Copyright (c) 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "nav2_behavior_tree/plugins/action/take_photo_action.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behavior_tree
{

TakePhotoAction::TakePhotoAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  image_received_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Get input parameters
  getInput("image_topic", image_topic_);
  getInput("save_dir", save_dir_);
  getInput("image_format", image_format_);
  getInput("filename_prefix", filename_prefix_);
  std::cout << "image_topic: " << image_topic_ << " save_dir: " << save_dir_ 
  << " image_format: " << image_format_ << " filename_prefix: " << filename_prefix_ << std::endl;

  // Ensure save directory exists
  try {
    if (!std::filesystem::exists(save_dir_)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Save directory does not exist: %s. Creating it...",
        save_dir_.c_str());
      if (!std::filesystem::create_directories(save_dir_)) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to create save directory: %s",
          save_dir_.c_str());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Exception while creating save directory: %s", e.what());
  }

  // Initialize current frame message
  curr_frame_msg_ = std::make_shared<sensor_msgs::msg::Image>();

  // Create image subscriber
  image_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TakePhotoAction::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "TakePhotoAction initialized. Subscribing to: %s, Saving to: %s",
    image_topic_.c_str(), save_dir_.c_str());
}

BT::NodeStatus TakePhotoAction::tick()
{ 
  std::cout << "tick" << std::endl;
  setStatus(BT::NodeStatus::RUNNING);

  // Check if image has been received
  // If not, return RUNNING to allow behavior tree to tick again
  if (!image_received_) {
    // Process any pending callbacks
    rclcpp::spin_some(node_);
    if (!image_received_) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "No image received yet from topic: %s. Will retry...",
        image_topic_.c_str());
      return BT::NodeStatus::RUNNING;
    }
  }

  try {
    // Generate filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << filename_prefix_
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count()
       << "." << image_format_;

    std::filesystem::path file_path = save_dir_ / ss.str();
    std::cout << "try save" << std::endl;

    // Save the image
    std::lock_guard<std::mutex> lock(image_mutex_);
    cv::Mat curr_frame_mat;
    deepCopyMsg2Mat(curr_frame_msg_, curr_frame_mat);

    if (curr_frame_mat.empty()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to convert image message to OpenCV Mat");
      return BT::NodeStatus::FAILURE;
    }

    if (!cv::imwrite(file_path.c_str(), curr_frame_mat)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to save image to: %s",
        file_path.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Photo saved successfully to: %s",
      file_path.c_str());

    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Exception while taking photo: %s",
      e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void TakePhotoAction::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  curr_frame_msg_ = msg;
  image_received_ = true;
  std::cout << "takephoto imageCallback" << std::endl;
}

void TakePhotoAction::deepCopyMsg2Mat(
  const sensor_msgs::msg::Image::SharedPtr & msg,
  cv::Mat & mat)
{
  try {
    cv_bridge::CvImageConstPtr cv_bridge_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat frame = cv_bridge_ptr->image;

    // Convert RGB to BGR if needed (OpenCV uses BGR by default)
    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    frame.copyTo(mat);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "cv_bridge exception: %s",
      e.what());
    mat = cv::Mat();
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TakePhotoAction>("TakePhoto");
}

