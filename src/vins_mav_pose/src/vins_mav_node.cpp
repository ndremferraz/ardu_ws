#include <functional>
#include <memory>

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
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose", 10);
    subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ov_srvins/poseimu", 10, std::bind(&PoseForwardNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose.pose;

    publisher_->publish(pose_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseForwardNode>());
  rclcpp::shutdown();
  return 0;
}
