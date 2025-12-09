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

#ifndef NAV2_CORE__BEHAVIOR_HPP_
#define NAV2_CORE__BEHAVIOR_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"

namespace nav2_core
{

/**
 * @class Behavior
 * @brief 行为抽象类，作为行为插件的接口
 */
class Behavior
{
public:
  using Ptr = std::shared_ptr<Behavior>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Behavior() {}

  /**
   * @brief 参数初始化行为
   * @param  parent 用户节点指针
   * @param  name 规划器名称
   * @param  tf tf缓冲指针
   * @param  costmap_ros 代价地图指针
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) = 0;

  /**
   * @brief 在关闭时清理行为使用的任何资源
   */
  virtual void cleanup() = 0;

  /**
   * @brief 激活行为及其执行的任何线程
   */
  virtual void activate() = 0;

  /**
   * @brief 停用行为及其执行的任何线程
   */
  virtual void deactivate() = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__BEHAVIOR_HPP_
