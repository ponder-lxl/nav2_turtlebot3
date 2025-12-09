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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TAKE_PHOTO_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TAKE_PHOTO_ACTION_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <filesystem>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to take a photo when robot reaches the goal
 */
class TakePhotoAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::TakePhotoAction constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TakePhotoAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("image_topic", "/camera/image_raw", "Camera image topic"),
      BT::InputPort<std::string>("save_dir", "/home/rx01109/loong_nav/picture", "Directory to save photos"),
      BT::InputPort<std::string>("image_format", "png", "Image format (png, jpg, etc.)"),
      BT::InputPort<std::string>("filename_prefix", "photo_", "Prefix for saved image filename"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Callback function for camera image subscription
   * @param msg Image message from camera
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Convert sensor_msgs::Image to OpenCV Mat
   * @param msg Image message
   * @param mat Output OpenCV Mat
   */
  void deepCopyMsg2Mat(const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & mat);

  // ROS node
  rclcpp::Node::SharedPtr node_;

  // Image subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  // Current frame message
  sensor_msgs::msg::Image::SharedPtr curr_frame_msg_;

  // Mutex for thread safety
  std::mutex image_mutex_;

  // Configuration parameters
  std::string image_topic_ = "/camera/image_raw";
  std::filesystem::path save_dir_ = "/home/rx01109/loong_nav/picture";
  std::string image_format_ = "png";
  std::string filename_prefix_ = "photo_";

  // Flag to check if image has been received
  bool image_received_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TAKE_PHOTO_ACTION_HPP_

