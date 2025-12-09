#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <string>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <cstring>

class NavPhotoNode : public rclcpp::Node
{
public:
  NavPhotoNode() : Node("nav_photo_node")
  {
  // 可配置的参数：状态话题、图像话题、保存目录
  this->declare_parameter<std::string>("status_topic", "/navigate_to_pose/_action/status");
  this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
  this->declare_parameter<std::string>("save_dir", "/home/rx01109/loong_nav/picture");

  std::string status_topic = this->get_parameter("status_topic").as_string();
  std::string image_topic = this->get_parameter("image_topic").as_string();
  save_dir_ = this->get_parameter("save_dir").as_string();

  ensureDirectoryExists(save_dir_);

  // 订阅导航状态（默认使用 nav2 的 navigate_to_pose action 状态话题）
  status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
    status_topic, 10, std::bind(&NavPhotoNode::statusCallback, this, std::placeholders::_1));

  // 订阅相机
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, 10, std::bind(&NavPhotoNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialized NavPhotoNode (status_topic=%s, image_topic=%s, save_dir=%s)",
        status_topic.c_str(), image_topic.c_str(), save_dir_.c_str());
  }

  void ensureDirectoryExists(const std::string &path)
  {
    struct stat sb;
    if (stat(path.c_str(), &sb) == 0)
    {
      if (!S_ISDIR(sb.st_mode))
      {
        RCLCPP_WARN(this->get_logger(), "%s exists but is not a directory", path.c_str());
      }
      return;
    }

    if (mkdir(path.c_str(), 0755) != 0)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to create directory %s: %s", path.c_str(), std::strerror(errno));
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Created directory %s", path.c_str());
    }
  }

private:
  void statusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "statusCallback running, entries=%zu", msg->status_list.size());
    for (auto & goal : msg->status_list)
    {
      if (goal.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
        // 根据 goal_id 去重，避免同一个 goal 重复保存多次
        std::ostringstream oss;
        for (auto b : goal.goal_info.goal_id.uuid)
        {
          oss << std::hex << std::setw(2) << std::setfill('0') << (int)b;
        }
        std::string gid = oss.str();

        {
          std::lock_guard<std::mutex> lk(id_mutex_);
          if (gid == last_succeeded_goal_id_)
          {
            RCLCPP_DEBUG(this->get_logger(), "Already handled goal %s, skipping", gid.c_str());
            break;
          }
          last_succeeded_goal_id_ = gid;
        }

        RCLCPP_INFO(this->get_logger(), "Navigation succeeded for goal %s, saving photo...", gid.c_str());
        saveLastImage();
        break;  // 只保存一次（每次回调保存一个成功目标）
      }
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
  cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  std::lock_guard<std::mutex> lk(image_mutex_);
  last_image_ = img;
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void saveLastImage()
  {
    RCLCPP_INFO(this->get_logger(), "Image save running");
    cv::Mat image_copy;
    {
      std::lock_guard<std::mutex> lk(image_mutex_);
      if (last_image_.empty())
      {
        RCLCPP_WARN(this->get_logger(), "No image received yet!");
        return;
      }
      image_copy = last_image_.clone();
      // 清空缓存图像，避免重复保存
      last_image_ = cv::Mat();
    }

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char buffer[64];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S.jpg", &tm);

    std::string save_path = save_dir_ + "/" + buffer;

    if (!cv::imwrite(save_path, image_copy))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to write image to: %s", save_path.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Image saved to: %s", save_path.c_str());
  }

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  cv::Mat last_image_;
  std::mutex image_mutex_;
  std::mutex id_mutex_;
  std::string last_succeeded_goal_id_;
  std::string save_dir_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavPhotoNode>());
  rclcpp::shutdown();
  return 0;
}
