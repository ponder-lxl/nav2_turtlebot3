// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_CORE__GLOBAL_PLANNER_HPP_
#define NAV2_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_core
{

/**
 * @class GlobalPlanner
 * @brief 全局规划抽象类，用于插件式的全局规划器
 */
class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;

  /**
   * @brief Virtual destructor
   */
  virtual ~GlobalPlanner() {}

  /**
   * @brief 初始化规划器
   * @param  parent 用户节点指针
   * @param  name 规划器名称
   * @param  tf tf缓冲指针
   * @param  costmap_ros 代价地图指针
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  /**
   * @brief 用于规划器关闭时清理规划器使用的任何资源
   */
  virtual void cleanup() = 0;

  /**
   * @brief 激活规划器及其执行的任何线程
   */
  virtual void activate() = 0;

  /**
   * @brief 停用规划器及其执行的任何线程
   */
  virtual void deactivate() = 0;

  /**
   * @brief 从起始姿态创建一个到目标姿态的规划方案
   * @param start 起始位置
   * @param goal  目标位置
   * @return      全局路径
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GLOBAL_PLANNER_HPP_
