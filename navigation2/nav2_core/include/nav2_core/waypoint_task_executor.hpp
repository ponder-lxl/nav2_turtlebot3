// Copyright (c) 2020 Fetullah Atas
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


#ifndef NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#define NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_core
{
/**
 * @brief 用于创建插件以在达到指定路径点时执行特定任务的抽象基类
 *
 */
class WaypointTaskExecutor
{
public:
  WaypointTaskExecutor() {}

  virtual ~WaypointTaskExecutor() {}

  /**
   * @brief 初始化
   * @param parent parent node that plugin will be created within(for an example see nav_waypoint_follower)
   * @param plugin_name plugin name comes from parameters in yaml file
   */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;

  /**
   * @brief 定义在到达指定点时执行任务的具体实现
   * @param curr_pose 当前机器人位置
   * @param curr_waypoint_index 当前指定点的索引
   * @return true表示任务执行成功
   */
  virtual bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index) = 0;
};
}  // namespace nav2_core
#endif  // NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
