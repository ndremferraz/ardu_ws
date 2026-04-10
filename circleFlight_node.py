import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class CircleFlight(Node):
    def __init__(self):
        super().__init__('circle_flight')

        # Publisher to MAVROS setpoint position
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_setpoint)

        # Circle parameters
        self.radius = 5.0        # meters
        self.altitude = 3.0      # meters
        self.angular_speed = 0.2 # rad/sec

        self.theta = 0.0

    def publish_setpoint(self):
        msg = PoseStamped()

        # Update angle
        self.theta += self.angular_speed * 0.05

        # Circle parametric equations
        x = self.radius * math.cos(self.theta)
        y = self.radius * math.sin(self.theta)
        z = self.altitude

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # Keep orientation fixed
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleFlight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()