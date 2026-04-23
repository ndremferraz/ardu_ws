import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

import math


class GoalToCmdVel(Node):
    def __init__(self):
        super().__init__('goal_to_cmd_vel')

        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose', self.pose_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        self.goal_reached_pub = self.create_publisher(
            Bool,
            '/goal_reached',
            10
        )

        self.current_pose = None
        self.goal_pose = None

        self.goal_active = False
        self.stop_sent = False

        #Parameters
        self.k_lin = 0.4
        self.lin_goal_tolerance = 0.25
        self.max_lin = 0.4

        # separate vertical gain
        self.ver_goal_tolerance = 0.1
        self.k_z = 0.2         
        self.max_z = 0.1

        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.goal_active = True
        self.stop_sent = False

        # Notify that we are working on a goal
        self.publish_goal_reached(False)

    def publish_stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def publish_goal_reached(self, reached: bool):
        msg = Bool()
        msg.data = reached
        self.goal_reached_pub.publish(msg)
        
    def control_loop(self):
        # No goal or no current pose -> just hold
        if self.current_pose is None or self.goal_pose is None:
            return

        if not self.goal_active:
            #self.publish_stop() need to add back if stopping after reaching goal
            return

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        dz = self.goal_pose.position.z - self.current_pose.position.z

        lin_distance = math.sqrt(dx * dx + dy * dy)

        # Stop condition (but DO NOT exit system)
        # Goal reached → send stop ONCE
        if lin_distance < self.lin_goal_tolerance and abs(dz) < self.ver_goal_tolerance:
            if not self.stop_sent:
                self.publish_stop()
                self.publish_goal_reached(True)
                self.stop_sent = True
            self.goal_active = False
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        vx = self.k_lin * dx
        vy = self.k_lin * dy
        vz = self.k_z * dz

        cmd.twist.linear.x = max(-self.max_lin, min(self.max_lin, vx))
        cmd.twist.linear.y = max(-self.max_lin, min(self.max_lin, vy))
        cmd.twist.linear.z = max(-self.max_z, min(self.max_z, vz))

        # NO yaw movement
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
