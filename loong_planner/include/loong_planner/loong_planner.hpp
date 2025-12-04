#ifndef LOONG_PLANNER_HPP_
#define LOONG_PLANNER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace loong_planner
{

class LoongPlanner : public nav2_core::Controller
{
public:
  LoongPlanner() = default;
  ~LoongPlanner() override = default;

  // 生命周期管理接口
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // LocalPlanner 核心接口
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  bool isGoalReached();

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("loongPlanner")};
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

  geometry_msgs::msg::TwistStamped last_cmd_vel_;
  nav_msgs::msg::Path global_plan_;
  size_t target_index_ = 0;
  bool pose_adjusting_ = false;
  bool goal_reached_ = false;

  // 参数
  double max_linear_velocity_x_ = 0.3;
  double max_linear_velocity_y_ = 0.2;
  double max_angular_velocity_ = 0.6;
  double linear_velocity_weight_x_ = 1.0;
  double linear_velocity_weight_y_ = 1.0;
  double angular_velocity_weight_ = 1.0;
  double path_follow_ = 0.2;
  double final_dist_ = 0.2;
  double velocity_smoothing_alpha_ = 0.4;
};

}  // namespace loong_planner

#endif  // LOONG_PLANNER_HPP_
