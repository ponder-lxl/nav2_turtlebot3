#include "loong_planner/loong_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace loong_planner
{

void LoongPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;

  tf_buffer_ = tf;
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  node->declare_parameter(plugin_name_ + ".max_linear_velocity_x", 0.3);
  node->declare_parameter(plugin_name_ + ".max_linear_velocity_y", 0.2);
  node->declare_parameter(plugin_name_ + ".max_angular_velocity", 0.6);
  node->declare_parameter(plugin_name_ + ".linear_velocity_weight_x", 1.0);
  node->declare_parameter(plugin_name_ + ".linear_velocity_weight_y", 1.0);
  node->declare_parameter(plugin_name_ + ".angular_velocity_weight", 1.0);
  node->declare_parameter(plugin_name_ + ".path_follow", 0.2);
  node->declare_parameter(plugin_name_ + ".final_dist", 0.2);
  node->declare_parameter(plugin_name_ + ".velocity_smoothing_alpha", 0.4);

  // 参数声明与获取
  node->get_parameter(plugin_name_ + "max_linear_velocity_x", max_linear_velocity_x_);
  node->get_parameter(plugin_name_ + "max_linear_velocity_y", max_linear_velocity_y_);
  node->get_parameter(plugin_name_ + "max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + "linear_velocity_weight_x", linear_velocity_weight_x_);
  node->get_parameter(plugin_name_ + "linear_velocity_weight_y", linear_velocity_weight_y_);
  node->get_parameter(plugin_name_ + "angular_velocity_weight", angular_velocity_weight_);
  node->get_parameter(plugin_name_ + "path_follow", path_follow_);
  node->get_parameter(plugin_name_ + "final_dist", final_dist_);
  node->get_parameter(plugin_name_ + "velocity_smoothing_alpha", velocity_smoothing_alpha_);

  last_cmd_vel_ = geometry_msgs::msg::TwistStamped();
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

  RCLCPP_INFO(rclcpp::get_logger("loong_planner"), "LoongPlanner configured!");
}

void LoongPlanner::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type loongplanner::lonngplanner",
    plugin_name_.c_str());
  global_pub_.reset();
}

void LoongPlanner::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type loongplanner::lonngplanner\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
  global_pub_->on_activate();
}

void LoongPlanner::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type loongplanner::lonngplanner\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
  global_pub_->on_deactivate();
}

void LoongPlanner::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  (void) speed_limit;
  (void) percentage;
}

void LoongPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  target_index_ = 0;
  pose_adjusting_ = false;
  goal_reached_ = false;
  global_pub_->publish(path);
}

geometry_msgs::msg::TwistStamped LoongPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) {
  auto cmd_vel = geometry_msgs::msg::TwistStamped();
  auto node = node_.lock();
  cmd_vel.header.stamp = node->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  std::cout << "global_plan_.poses.size(): " << global_plan_.poses.size() << std::endl;
  if (global_plan_.poses.empty()) {
    last_cmd_vel_ = cmd_vel;
    return cmd_vel;
  }

  nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  unsigned char *map_data = costmap->getCharMap();
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  // 全局路径前10个点碰撞检测
  for (size_t i = 0; i < global_plan_.poses.size(); i++)
  {
    geometry_msgs::msg::PoseStamped pose_ci2d;
    try {
      tf_buffer_->transform(global_plan_.poses[i], pose_ci2d, "map", tf2::durationFromSec(0.1));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "TF transform failed: %s", ex.what());
      cmd_vel.twist = geometry_msgs::msg::Twist();
      last_cmd_vel_ = cmd_vel;
      return cmd_vel;
    }
    double odom_x = pose_ci2d.pose.position.x;
    double odom_y = pose_ci2d.pose.position.y;

    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();
    int x = static_cast<int>((odom_x - origin_x) / costmap->getResolution());
    int y = static_cast<int>((odom_y - origin_y) / costmap->getResolution());

    if (i >= static_cast<size_t>(target_index_) && i < static_cast<size_t>(target_index_ + 10))
    {
      if (x < 0 || y < 0 || x >= static_cast<int>(size_x) || y >= static_cast<int>(size_y))
        continue;

      int map_index = y * size_x + x;
      unsigned char cost = map_data[map_index];
      if (cost >= 253)
      {
        cmd_vel.twist = geometry_msgs::msg::Twist();
        last_cmd_vel_ = cmd_vel;
        std::cout << "Path blocked at index: " << i << ", cost: " << static_cast<int>(cost) << std::endl;
        return cmd_vel;
      }
    }
  }

  geometry_msgs::msg::PoseStamped pose_final;
  try {
    tf_buffer_->transform(global_plan_.poses.back(), pose_final, "base_link", tf2::durationFromSec(0.1));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "TF to body_2d failed: %s", ex.what());
    cmd_vel.twist = geometry_msgs::msg::Twist();
    last_cmd_vel_ = cmd_vel;
    return cmd_vel;
  }

  double dx = pose_final.pose.position.x;
  double dy = pose_final.pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  double final_yaw = tf2::getYaw(pose_final.pose.orientation);
  double delta_yaw = final_yaw;
  if (delta_yaw > M_PI)
      delta_yaw -= 2 * M_PI;
  else if (delta_yaw < -M_PI)
      delta_yaw += 2 * M_PI;

  if (!pose_adjusting_)
  {
    bool near_tail = (target_index_ >= static_cast<int>(global_plan_.poses.size()) - 5);
    if (dist < final_dist_ || near_tail)
    {
      pose_adjusting_ = true;
      last_cmd_vel_.twist = geometry_msgs::msg::Twist();
    }
  }

  if (pose_adjusting_) {
    if (dist < final_dist_ && std::fabs(delta_yaw) < 0.25) {
      goal_reached_ = true;
      cmd_vel.twist = geometry_msgs::msg::Twist();
      last_cmd_vel_ = cmd_vel;
      RCLCPP_INFO(rclcpp::get_logger("loong_planner"), "Goal reached!");
      return cmd_vel;
    }
    else {
      double linear_velocity_x = std::clamp(dx, -max_linear_velocity_x_, max_linear_velocity_x_);
      double linear_velocity_y = std::clamp(dy, -max_linear_velocity_y_, max_linear_velocity_y_);
      double angular_velocity = std::clamp(1.5 * delta_yaw, -max_angular_velocity_, max_angular_velocity_);

      cmd_vel.twist.linear.x = linear_velocity_x;
      cmd_vel.twist.linear.y = linear_velocity_y;
      cmd_vel.twist.angular.z = angular_velocity;
      last_cmd_vel_ = cmd_vel;
      return cmd_vel;
    }
  }

  // 选择目标点，开始跟踪
  geometry_msgs::msg::PoseStamped target_pose;
  for (size_t i = target_index_; i < global_plan_.poses.size(); i++)
  {
    geometry_msgs::msg::PoseStamped pose_base;
    try {
      tf_buffer_->transform(global_plan_.poses[i], pose_base, "base_link", tf2::durationFromSec(0.1));
    }
    catch (tf2::TransformException &) {
      continue;
    }

    double tx = pose_base.pose.position.x;
    double ty = pose_base.pose.position.y;
    double tdist = std::sqrt(tx * tx + ty * ty);

    if (tdist > path_follow_)
    {
      target_pose = pose_base;
      target_index_ = i;
      break;
    }
    if (i == global_plan_.poses.size() - 1)
    {
      target_pose = pose_base;
    }
  }

  bool is_last_points = (target_index_ >= static_cast<int>(global_plan_.poses.size()) - 16);
  double lvx = linear_velocity_weight_x_ * target_pose.pose.position.x;
  double lvy = linear_velocity_weight_y_ * target_pose.pose.position.y;
  double av = angular_velocity_weight_ * target_pose.pose.position.y;

  if (is_last_points)
  {
    lvx *= 0.4;
    lvy *= 0.4;
    av *= 0.4;
  }

  lvx = std::clamp(lvx, -max_linear_velocity_x_, max_linear_velocity_x_);
  lvy = std::clamp(lvy, -max_linear_velocity_y_, max_linear_velocity_y_);
  av = std::clamp(av, -max_angular_velocity_, max_angular_velocity_);

  cmd_vel.twist.linear.x  = velocity_smoothing_alpha_ * lvx + (1.0 - velocity_smoothing_alpha_) * last_cmd_vel_.twist.linear.x;
  cmd_vel.twist.linear.y  = velocity_smoothing_alpha_ * lvy + (1.0 - velocity_smoothing_alpha_) * last_cmd_vel_.twist.linear.y;
  cmd_vel.twist.angular.z = velocity_smoothing_alpha_ * av  + (1.0 - velocity_smoothing_alpha_) * last_cmd_vel_.twist.angular.z;

  last_cmd_vel_ = cmd_vel;
  return cmd_vel;
}

bool LoongPlanner::isGoalReached()
{
  return goal_reached_;
}

}  // namespace loong_planner

PLUGINLIB_EXPORT_CLASS(loong_planner::LoongPlanner, nav2_core::Controller)
