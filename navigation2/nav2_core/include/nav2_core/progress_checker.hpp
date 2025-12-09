// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_CORE__PROGRESS_CHECKER_HPP_
#define NAV2_CORE__PROGRESS_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_core
{
/**
 * @class nav2_core::ProgressChecker
 * @brief 机器人到目标点过程检测类
 */
class ProgressChecker
{
public:
  typedef std::shared_ptr<nav2_core::ProgressChecker> Ptr;

  virtual ~ProgressChecker() {}

  /**
   * @brief 参数初始化
   * @param parent 生命周期节点指针
   * @param plugin_name 插件名称
   */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;
  /**
   * @brief 过程状态检测
   * @param current_pose 机器人当前位置
   * @return 有进展返回true
   */
  virtual bool check(geometry_msgs::msg::PoseStamped & current_pose) = 0;
  /**
   * @brief 参数重置
   */
  virtual void reset() = 0;
};
}  // namespace nav2_core

#endif  // NAV2_CORE__PROGRESS_CHECKER_HPP_
