#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class PoseToOdom : public rclcpp::Node {
public:
    PoseToOdom() : Node("pose_to_odom") {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose",
            10,
            std::bind(&PoseToOdom::poseCallback, this, std::placeholders::_1)
        );
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        nav_msgs::msg::Odometry odom;

        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position = msg->pose.position;
        odom.pose.pose.orientation = msg->pose.orientation;

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_footprint";

        transform.transform.translation.x = msg->pose.position.x;
        transform.transform.translation.y = msg->pose.position.y;
        transform.transform.translation.z = msg->pose.position.z;
        transform.transform.rotation = msg->pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseToOdom>());
    rclcpp::shutdown();
    return 0;
}
