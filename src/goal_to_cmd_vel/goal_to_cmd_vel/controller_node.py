import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

import math


class GoalToCmdVel(Node):
    def __init__(self):
        super().__init__('goal_to_cmd_vel')

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        self.current_pose = None
        self.goal_pose = None

        self.goal_active = False  # NEW: state flag

        self.k_lin = 0.4
        self.goal_tolerance = 0.25
        self.max_lin = 0.4

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.goal_active = True  # NEW goal received → activate control

    def publish_stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def control_loop(self):
        # No goal or no odom → just hold
        if self.current_pose is None or self.goal_pose is None:
            self.publish_stop()
            return

        if not self.goal_active:
            self.publish_stop()
            return

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y

        distance = math.sqrt(dx * dx + dy * dy)

        # Stop condition (but DO NOT exit system)
        if distance < self.goal_tolerance:
            self.publish_stop()
            self.goal_active = False  # enter idle mode
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        vx = self.k_lin * dx
        vy = self.k_lin * dy

        cmd.twist.linear.x = max(-self.max_lin, min(self.max_lin, vx))
        cmd.twist.linear.y = max(-self.max_lin, min(self.max_lin, vy))
        cmd.twist.linear.z = 0.0

        # NO yaw control
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()