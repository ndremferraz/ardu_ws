import time 
import os

import vicon_tracker 
import rclpy
import rclpy.node
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped


class ViconPublisher(rclpy.node.Node):

    def __init__(self):
        super().__init__('vicon_publisher')

        self.declare_parameter('config', 'config.cfg')

        timer_period = 0.033  # seconds

        config_file = self.get_parameter('config').get_parameter_value().string_value 
        object_name = self.load_config(config_file)
        
        self.vicon = vicon_tracker.vicon()
        self.vicon.open(object_name)

        self.publisher_ = self.create_publisher(PoseStamped, '/vicon/pose', 3)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pose_msg = PoseStamped()

        x_v, R_vm = self.vicon.loop()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'vicon_frame'

        pose_msg.pose.position.x = x_v[0]
        pose_msg.pose.position.y = x_v[1]
        pose_msg.pose.position.z = x_v[2]
        
        # Convert 3x3 rotation matrix to quaternion [x, y, z, w]
        q = R.from_matrix(R_vm).as_quat() 

        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.publisher_.publish(pose_msg)

    def load_config(self, config_file="config.cfg"):

        """Read the Vicon object name from config.cfg."""
        object_name = "OriginsX@192.168.11.2"  # Default fallback
        try:

            with open(config_file, 'r') as file:
                for line in file:
                    line = line.strip()
                    if line.startswith("object:"):
                        object_name = line.split("object:")[1].strip().strip('"')
                        break
            self.get_logger().info(f"Loaded Vicon object: {object_name}")

        except FileNotFoundError:

            self.get_logger().info(f"Config file {config_file} not found, using default object: {object_name}")
        except Exception as e:

            self.get_logger().info(f"Error reading config file: {e}, using default object: {object_name}")
        return object_name

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
