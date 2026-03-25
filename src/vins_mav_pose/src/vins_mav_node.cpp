#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class PoseForwardNode : public rclcpp::Node
{
public:
  PoseForwardNode()
      : Node("vins_mav_node"), max_buffer_size_(2000)
  {
    this->declare_parameter<std::string>("pose_source", "vins");

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    // Regular VINS
    subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ov_srvins/poseimu", 10, std::bind(&PoseForwardNode::topic_callback, this, _1));
    // Vicon subscription
    vicon_subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vicon/pose", 10, std::bind(&PoseForwardNode::vicon_callback, this, _1));

    stats_timer_ = this->create_wall_timer(
        std::chrono::seconds(5), std::bind(&PoseForwardNode::log_stats, this));
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose.pose;

    // Record data
    record_pose(vins_history_, pose_msg);

    // Forward if active
    if (this->get_parameter("pose_source").as_string() == "vins")
    {
      publisher_->publish(pose_msg);
    }
  }
  // Callback for Vicon data
  void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    // Publish the message directly
    record_pose(vicon_history_, pose_msg);

    if (this->get_parameter("pose_source").as_string() == "vicon")
    {
      publisher_->publish(*msg);
    }
  }

  void log_stats() const
  {
    RCLCPP_INFO(this->get_logger(), "History Status -> VINS: %zu, Vicon: %zu points",
                vins_history_.size(), vicon_history_.size());
  }

  void record_pose(std::vector<geometry_msgs::msg::PoseStamped> &history, const geometry_msgs::msg::PoseStamped &msg)
  {
    history.push_back(msg);
    if (history.size() > max_buffer_size_)
    {
      history.erase(history.begin()); // Remove oldest
    }
  }
  size_t max_buffer_size_;
  std::vector<geometry_msgs::msg::PoseStamped> vins_history_;
  std::vector<geometry_msgs::msg::PoseStamped> vicon_history_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseForwardNode>());
  rclcpp::shutdown();
  return 0;
}
