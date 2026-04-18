#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class VelPub(Node):
    def __init__(self):
        super().__init__('vel_pub')

        self.pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        self.timer = self.create_timer(0.1, self.send_cmd)  # 10 Hz

    def send_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = -0.3
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = VelPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()