#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import OverrideRCIn, RCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


PI_2 = math.pi / 2.0


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """
    Convert Euler angles to quaternion.
    """
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


class MavController(Node):
    """
    A simple ROS 2 object to help interface with MAVROS.
    """

    def __init__(self):
        super().__init__("mav_control_node")

        self.pose = Pose()
        self.last_stamp = self.get_clock().now().to_msg()

        self.cmd_pos_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )

        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        self.get_logger().info("Waiting for MAVROS services...")
        self.mode_client.wait_for_service()
        self.arm_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.get_logger().info("MAVROS services are available.")


    def goto(self, pose: Pose):
        """
        Publish a position setpoint. Vehicle should be in GUIDED mode.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = pose
        self.cmd_pos_pub.publish(pose_stamped)


    def _call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Service call failed.")
            return None
        return future.result()

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        resp = self._call_service(self.arm_client, req)
        if resp is not None:
            self.get_logger().info(f"Arm response: success={resp.success}")
        return resp

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        resp = self._call_service(self.arm_client, req)
        if resp is not None:
            self.get_logger().info(f"Disarm response: success={resp.success}")
        return resp

    def set_mode(self, mode_name: str):
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_name
        resp = self._call_service(self.mode_client, req)
        if resp is not None:
            self.get_logger().info(f"Set mode '{mode_name}': mode_sent={resp.mode_sent}")
        return resp

    def takeoff(self, height=1.0):
        """
        Set GUIDED mode, arm, then issue takeoff command.
        """
        self.set_mode("GUIDED")
        self.arm()

        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = float(height)

        resp = self._call_service(self.takeoff_client, req)
        if resp is not None:
            self.get_logger().info(f"Takeoff response: success={resp.success}")
        return resp

    def land(self):
        """
        Put vehicle in LAND mode.
        """
        return self.set_mode("LAND")


def ros2_sleep(node: Node, executor: SingleThreadedExecutor, seconds: float):
    """
    Sleep while still spinning callbacks.
    """
    end_time = time.time() + seconds
    while rclpy.ok() and time.time() < end_time:
        executor.spin_once(timeout_sec=0.1)


def simple_demo():
    rclpy.init()

    controller = MavController()
    executor = SingleThreadedExecutor()
    executor.add_node(controller)

    try:
        ros2_sleep(controller, executor, 1.0)

        controller.get_logger().info("Takeoff")
        controller.takeoff(0.5)
        ros2_sleep(controller, executor, 3.0)

        controller.goto_xyz_rpy(0.0, 0.0, 1.2, 0.0, 0.0, 0.0)
        ros2_sleep(controller, executor, 3.0)

        controller.get_logger().info("Waypoint 1: position control")
        controller.goto_xyz_rpy(0.0, 0.0, 1.2, 0.0, 0.0, -1 * PI_2)
        ros2_sleep(controller, executor, 2.0)
        controller.goto_xyz_rpy(0.4, 0.0, 1.2, 0.0, 0.0, -1 * PI_2)
        ros2_sleep(controller, executor, 3.0)

        controller.get_logger().info("Waypoint 2: position control")
        controller.goto_xyz_rpy(0.4, 0.0, 1.2, 0.0, 0.0, 0.0)
        ros2_sleep(controller, executor, 2.0)
        controller.goto_xyz_rpy(0.4, 0.4, 1.2, 0.0, 0.0, 0.0)
        ros2_sleep(controller, executor, 3.0)

        controller.get_logger().info("Waypoint 3: position control")
        controller.goto_xyz_rpy(0.4, 0.4, 1.2, 0.0, 0.0, PI_2)
        ros2_sleep(controller, executor, 2.0)
        controller.goto_xyz_rpy(0.0, 0.4, 1.2, 0.0, 0.0, PI_2)
        ros2_sleep(controller, executor, 3.0)

        controller.get_logger().info("Waypoint 4: position control")
        controller.goto_xyz_rpy(0.0, 0.4, 1.2, 0.0, 0.0, 2 * PI_2)
        ros2_sleep(controller, executor, 2.0)
        controller.goto_xyz_rpy(0.0, 0.0, 1.2, 0.0, 0.0, 2 * PI_2)
        ros2_sleep(controller, executor, 3.0)

        controller.get_logger().info("Landing")
        controller.land()
        ros2_sleep(controller, executor, 2.0)

    finally:
        executor.remove_node(controller)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    simple_demo()