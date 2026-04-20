#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class PoseForwardNode : public rclcpp::Node
{
public:
  PoseForwardNode()
      : Node("vins_mav_node")
  {
    this->declare_parameter<std::string>("pose_source", "vins");

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    // Regular VINS
    subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/zed/zed_node/pose", 10, std::bind(&PoseForwardNode::vins_callback, this, _1));
    // Vicon subscription
    vicon_subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vicon/pose", 10, std::bind(&PoseForwardNode::vicon_callback, this, _1));


  }

  ~PoseForwardNode(){

    std::ofstream vicon_vs_vins_file("vins_history.csv");

    vicon_vs_vins_file
      << "source,stamp_sec,stamp_nanosec,x,y,z,qx,qy,qz,qw\n";

    for (const auto& pose : vins_history_) {
      vicon_vs_vins_file
        << "vins,"
        << pose.header.stamp.sec << ","
        << pose.header.stamp.nanosec << ","
        << pose.pose.position.x << ","
        << pose.pose.position.y << ","
        << pose.pose.position.z << ","
        << pose.pose.orientation.x << ","
        << pose.pose.orientation.y << ","
        << pose.pose.orientation.z << ","
        << pose.pose.orientation.w << "\n";
    }

    for (const auto& pose : vicon_history_) {
      vicon_vs_vins_file
        << "vicon,"
        << pose.header.stamp.sec << ","
        << pose.header.stamp.nanosec << ","
        << pose.pose.position.x << ","
        << pose.pose.position.y << ","
        << pose.pose.position.z << ","
        << pose.pose.orientation.x << ","
        << pose.pose.orientation.y << ","
        << pose.pose.orientation.z << ","
        << pose.pose.orientation.w << "\n";
    }

    vicon_vs_vins_file.close();

  }

private:
  void vins_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose;

    vins_history_.push_back(pose_msg);

    if (this->get_parameter("pose_source").as_string() == "vins")
    {
      publisher_->publish(pose_msg);
    }
  }

  void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)  
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose;

    vicon_history_.push_back(pose_msg);

    if (this->get_parameter("pose_source").as_string() == "vicon")
    {
      publisher_->publish(pose_msg);
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> vins_history_;
  std::vector<geometry_msgs::msg::PoseStamped> vicon_history_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
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
