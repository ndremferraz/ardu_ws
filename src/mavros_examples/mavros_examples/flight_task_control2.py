#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class GuidedLocalTest(Node):
    def __init__(self):
        super().__init__('guided_local_test')

        self.state = State()

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10
        )

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        self.wait_for_services()
        self.get_logger().info('MAVROS services are ready.')

    def state_cb(self, msg: State):
        self.state = msg


    def wait_for_services(self):
        for client in [self.arming_client, self.set_mode_client]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service...')

    def arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().success

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().mode_sent

    def make_pose(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg

    def publish_setpoint_for(self, msg: PoseStamped, seconds: float, rate_hz: float = 20.0):
        period = 1.0 / rate_hz
        end_time = time.time() + seconds

        while rclpy.ok() and time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def run(self):
        self.get_logger().info('Waiting for FCU connection...')
        while rclpy.ok() and not self.state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('FCU connected.')

        # Prime the FCU with a stream of setpoints before switching to GUIDED.
        hold = self.make_pose(0.0, 0.0, 0.0)
        self.get_logger().info('Priming local setpoint stream...')
        self.publish_setpoint_for(hold, seconds=3.0)

        self.get_logger().info('Setting mode to GUIDED...')
        if not self.set_mode('GUIDED'):
            self.get_logger().error('Failed to set GUIDED.')
            return

        time.sleep(1.0)

        self.get_logger().info('Arming...')
        if not self.arm():
            self.get_logger().error('Failed to arm.')
            return

        time.sleep(1.0)

        # SITL-only example relative targets in local ENU-ish ROS pose topic.
        # Adjust signs only after verifying your simulator/frame conventions.
        targets = [
            (0.0, 0.0, 1.0, 5.0),   # rise
            (1.0, 0.0, 1.0, 5.0),   # move forward
            (1.0, 1.0, 1.0, 5.0),   # move sideways
            (0.0, 0.0, 1.0, 5.0),   # return near start
            (0.0, 0.0, 0.5, 4.0),   # descend a bit
        ]

        for x, y, z, duration in targets:
            self.get_logger().info(f'Publishing target x={x:.2f}, y={y:.2f}, z={z:.2f}')
            target = self.make_pose(x, y, z)
            self.publish_setpoint_for(target, seconds=duration)

        self.get_logger().info('Test sequence complete. Holding last setpoint for 3 seconds.')
        self.publish_setpoint_for(self.make_pose(0.0, 0.0, 0.5), seconds=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = GuidedLocalTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()