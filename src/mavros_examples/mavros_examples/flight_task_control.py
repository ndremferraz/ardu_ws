#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class TaskControl(Node):
    def __init__(self):
        super().__init__('task_control')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.pose = None
        self.last_pose_time = None

        self.get_logger().info('Waiting for MAVROS services...')
        for client in [
            self.arming_client,
            self.set_mode_client,
            self.takeoff_client,
            self.land_client,
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service...')
        self.get_logger().info('MAVROS services are ready!')

    def pose_callback(self, msg):
        self.pose = msg.pose
        self.last_pose_time = msg.header.stamp

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().success

    def set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().mode_sent

    def takeoff(self, altitude: float):
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().success

    def land(self):
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None and future.result().success

    def make_pose(self, x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def hold_waypoint(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, hold_time=5.0, rate_hz=20.0):
        pose = self.make_pose(x, y, z, roll, pitch, yaw)
        period = 1.0 / rate_hz
        end_time = time.time() + hold_time

        while time.time() < end_time:
            pose.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pos_pub.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = TaskControl()

    try:
        node.get_logger().info('Setting mode to GUIDED...')
        if not node.set_mode('GUIDED'):
            node.get_logger().error('Failed to set GUIDED mode')
            return

        time.sleep(1.0)

        node.get_logger().info('Arming...')
        if not node.arm():
            node.get_logger().error('Failed to arm')
            return

        time.sleep(1.0)

        node.get_logger().info('Taking off...')
        if not node.takeoff(1.5):
            node.get_logger().error('Failed to take off')
            return

        time.sleep(8.0)

        waypoints = [
            ('front', 1.0, 0.0, 1.5),
            ('origin', 0.0, 0.0, 1.5),
            ('back', -1.0, 0.0, 1.5),
            ('origin', 0.0, 0.0, 1.5),
            ('left', 0.0, 1.0, 1.5),
            ('origin', 0.0, 0.0, 1.5),
            ('right', 0.0, -1.0, 1.5),
        ]

        for name, x, y, z in waypoints:
            node.get_logger().info(f'Holding {name}: ({x}, {y}, {z})')
            node.hold_waypoint(x, y, z, hold_time=5.0, rate_hz=20.0)

        node.get_logger().info('Landing...')
        node.land()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()