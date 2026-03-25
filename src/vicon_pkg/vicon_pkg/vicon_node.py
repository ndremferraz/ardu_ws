import time 
import os

import vicon_tracker 
import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class ViconPublisher(Node):

    def __init__(self):
        super().__init__('vicon_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'vicon_topic', 3)
        timer_period = 0.033  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pose_msg = PoseStamped()

        x_v, R_vm = self.vicon.loop()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'vicon_frame'

        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 2.0
        pose_msg.pose.position.z = 3.0
        
        # R_vm to quaternion (use the following form x,y,z,w)
        
        # x_b = 
        # q = rot2quat(R_vm) # q = [x,y,z,w]

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ViconPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
