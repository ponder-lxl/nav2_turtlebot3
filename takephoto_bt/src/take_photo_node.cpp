#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <string>
#include <lifecycle_msgs/msg/transition_event.hpp>

class NavPhotoNode : public rclcpp::Node
{
public:
  NavPhotoNode() : Node("nav_photo_node")
  {
    // 订阅导航状态
    status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/bt_navigator/goal_status", 10,
            std::bind(&NavPhotoNode::statusCallback, this, std::placeholders::_1));

    // 订阅相机
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&NavPhotoNode::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Initialized NavPhotoNode");
  }

private:
  void statusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "statusCallback runnning...");
    for (auto & goal : msg->status_list)
    {
      if (goal.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded, saving photo...");
        saveLastImage();
        break;  // 只保存一次
      }
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      last_image_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
      // RCLCPP_INFO(this->get_logger(), "Image saved");
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void saveLastImage()
  {
    RCLCPP_INFO(this->get_logger(), "Image save running");
    if (last_image_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No image received yet!");
      return;
    }

    std::string save_dir = "/home/rx01109/loong_nav/picture";
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char buffer[64];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S.jpg", &tm);

    std::string save_path = save_dir + "/" + buffer;

    cv::imwrite(save_path, last_image_);
    RCLCPP_INFO(this->get_logger(), "Image saved to: %s", save_path.c_str());

    last_image_ = cv::Mat();  // 清空
  }

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  cv::Mat last_image_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavPhotoNode>());
  rclcpp::shutdown();
  return 0;
}
