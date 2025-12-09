/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CORE__CONTROLLER_HPP_
#define NAV2_CORE__CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/goal_checker.hpp"


namespace nav2_core
{

/**
 * @class Controller
 * @brief 导航控制器抽象类，作为导航控制器的接口
 */
class Controller
{
public:
  using Ptr = std::shared_ptr<nav2_core::Controller>;


  /**
   * @brief Virtual destructor
   */
  virtual ~Controller() {}

  /**
   * @brief 参数初始化
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;

  /**
   * @brief 清理控制器使用的资源
   */
  virtual void cleanup() = 0;

  /**
   * @brief 激活控制器及其执行的任何线程
   */
  virtual void activate() = 0;

  /**
   * @brief 停止控制器及其执行的任何线程
   */
  virtual void deactivate() = 0;

  /**
   * @brief 为控制器设置全局路径
   * @param path 全局路径
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief 根据当前位置和速度计算最佳控制指令
   * @param pose 当前位置
   * @param velocity 当前机器人的速度
   * @param goal_checker 正在使用的目标检查器指针
   * @return 最佳控制指令
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  /**
   * @brief 根据提供的速度限制和是否以绝对值或百分比形式表示，限制机器人的最大线速度
   * @param speed_limit 体现为绝对值的速度限制，或者百分比的速度限制
   * @param percentage true：使用速度限制百分比，false：使用绝对值限制
   */
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__CONTROLLER_HPP_
