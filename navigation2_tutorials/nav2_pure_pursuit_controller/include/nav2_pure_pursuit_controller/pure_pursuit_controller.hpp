/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#ifndef NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_pure_pursuit_controller
{

class PurePursuitController : public nav2_core::Controller
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;


  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
	/**
	 * @brief 
	 * 1. 将全局路径在局部地图范围内裁减，并根据当前位置对裁剪路径点作tf转化
	 * 2. 将距离当前位置最近的全局路径点之前的路径点删除，剩余的作为全局路径
	 * @return 返回的是局部地图裁剪并转化后的路径点
	 */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

	// outpose到in_pose的tf转化
  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance
  ) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double desired_linear_vel_; // 期望线速度
  double lookahead_dist_; // 预瞄距离
  double max_angular_vel_; // 最大角速度
  rclcpp::Duration transform_tolerance_ {0, 0};

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
};

}  // namespace nav2_pure_pursuit_controller

#endif  // NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_