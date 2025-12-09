// Copyright (c) 2021 RoboTech Vision
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

#ifndef NAV2_CORE__SMOOTHER_HPP_
#define NAV2_CORE__SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"


namespace nav2_core
{

/**
 * @class Smoother
 * @brief 路径平滑抽象类
 */
class Smoother
{
public:
  using Ptr = std::shared_ptr<nav2_core::Smoother>;

  virtual ~Smoother() {}

  /**
   * @brief 参数配置
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) = 0;

  /**
   * @brief 清理平滑器使用的资源
   */
  virtual void cleanup() = 0;

  /**
   * @brief 激活平滑器及其执行的任何线程
   */
  virtual void activate() = 0;

  /**
   * @brief 停止平滑器及其执行的任何线程
   */
  virtual void deactivate() = 0;

  /**
   * @brief 在指定的最大时间范围内平滑给定的路径
   *
   * @param path 被平滑的路径，原始路径
   * @param max_time 平滑最大使用时间
   * @return 返回bool值，表示平滑是否完成或因时间限制而中断
   */
  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__SMOOTHER_HPP_
