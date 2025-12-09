/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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

#ifndef NAV2_CORE__GOAL_CHECKER_HPP_
#define NAV2_CORE__GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"


namespace nav2_core
{

/**
 * @class GoalChecker
 * @brief 检测机器人是否到达目标
 */
class GoalChecker
{
public:
  typedef std::shared_ptr<nav2_core::GoalChecker> Ptr;  // 定义类型别名ptr，类型为指向GoalChecker的shared_ptr

  virtual ~GoalChecker() {}

  /**
   * @brief 用于从Nodehandle初始化任何参数
   * @param parent 用于获取参数的节点指针
   * @param plugin_name 插件名称 
   * @param costmap_ros 代价地图
   */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;
  
  // 重置GoalChecker对象的状态
  virtual void reset() = 0;

  /**
   * @brief 检查是否到达目标
   * @param query_pose 查询姿态
   * @param goal_pose 目标姿态
   * @param velocity 机器人当前速度
   * @return 返回是否达到
   */
  virtual bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) = 0;

  /**
   * @brief 获取用于目标检查的主要类型的最大可能公差
   * @param pose_tolerance 位置容差
   * @param vel_tolerance 速度容差
   * @return ture, 表示容差被合理使用
   */
  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GOAL_CHECKER_HPP_
