#!/usr/bin/env python3
import rclpy  # noqa: I100
import numpy as np
from geometry_msgs.msg import PoseStamped,TwistStamped  # noqa: F401,I100
from mavros_msgs.msg import State  # noqa: F401
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros.vel_pid import WaypointController

import math
import time


class TaskControl(Node):
    def __init__(self):
        super().__init__('task_control')

        self.declare_parameter('max_speed', 0.5)
        self.max_speed = self.get_parameter('max_speed').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        self.cmd_pos_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", qos)

        self.pose_sub = self.create_subscription(
            PoseStamped, "/vicon/pose", self.pose_callback, 5)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.current_posi = None
        self.initial_posi = None
        self.current_vel = np.array([0.0,0.0,0.0],dtype=float)

        self.controller = WaypointController(max_speed=self.max_speed)        

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

    def pose_callback(self, pose_msg):

        posi = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z], dtype=float)
        
        self.current_posi = posi

        if self.initial_posi is None:
            self.initial_posi = posi

    def arm(self) -> bool:
        """Arm the quadcopter."""
        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Vehicle armed successfully')
                return True
            else:
                self.get_logger().warn('Failed to arm vehicle')
                return False
        else:
            self.get_logger().error('Arming service call failed')
            return False

    def disarm(self) -> bool:
        """Disarm the quadcopter."""
        req = CommandBool.Request()
        req.value = False

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Vehicle disarmed successfully')
                return True
            else:
                self.get_logger().warn('Failed to disarm vehicle')
                return False
        else:
            self.get_logger().error('Disarming service call failed')
            return False

    def takeoff(self, altitude: float) -> bool:
        """
        Takeoff to specified altitude.

        :param altitude: in meters
        """
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0  # Use current position
        req.longitude = 0.0
        req.altitude = altitude

        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Takeoff command sent. Target altitude: {altitude}m')
                return True
            else:
                self.get_logger().warn('Takeoff command failed')
                return False
        else:
            self.get_logger().error('Takeoff service call failed')
            return False

    def land(self) -> bool:
        """Land the quadcopter."""
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Landing command sent')
                return True
            else:
                self.get_logger().warn('Landing command failed')
                return False
        else:
            self.get_logger().error('Land service call failed')
            return False

    def set_mode(self, mode: str) -> bool:
        """
        Set flight mode.

        Common modes: STABILIZED, GUIDED, RTL, LAND
        """
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Mode set to {mode}')
                return True
            else:
                self.get_logger().warn(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error('Set mode service call failed')
            return False


    def goto_xyz(self, xyz, period_sec=0.075):
        '''
        Moves to xyz coordinates by using PID to control velocity
        '''


        t0 = self.get_clock().now()

        xyz = xyz + self.initial_posi
        reached = False

        while rclpu.ok() and not reached:
            dt = (self.get_clock().now() - t0).nanoseconds * 1e-9

            if self.current_posi is not None:
                reached, cmd_vel = self.controller.update(self.current_posi, self.current_vel, xyz, dt)
                self.send_cmd_vel(cmd_vel)
                self.current_vel = cmd_vel.copy()

            t0 = self.get_clock().now()
            self.ros_sleep(period_sec)

    def send_cmd_vel(self, cmd_vel):
        '''
        Sends command velocity using command velocity topic
        '''

        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'map'

        vel_msg.twist.linear.x = cmd_vel[0]
        vel_msg.twist.linear.y = cmd_vel[1]
        vel_msg.twist.linear.z = cmd_vel[2]
        vel_msg.twist.angular.z = 0.0

        self.cmd_pos_pub.publish(vel_msg)


    def ros_sleep(self, duration_sec):

        end = time.monotonic() + duration_sec

        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_initial_position(self, timeout_sec=5.0):
        start = time.monotonic()

        while rclpy.ok() and self.initial_posi is None:
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.monotonic() - start > timeout_sec:
                return False

        return True


def main(args=None):
    rclpy.init(args=args)

    try:
        task_control = TaskControl()

        task_control.get_logger().info('=== Example 1: Basic Flight Sequence ===')

        task_control.ros_sleep(2.0)


        task_control.get_logger().info('Waiting for initial Vicon position...')
        if not task_control.wait_for_initial_position(timeout_sec=5.0):
            task_control.get_logger().error('Did not receive initial position. Exiting without takeoff.')
            return
        

        # Set to GUIDED mode
        task_control.get_logger().info('Setting mode to GUIDED...')
        if not task_control.set_mode('GUIDED'):
            task_control.get_logger().error('Failed to set GUIDED mode. Exiting...')
            return
        task_control.ros_sleep(2.0)

        # Arm the vehicle
        task_control.get_logger().info('Arming the vehicle...')
        if not task_control.arm():
            task_control.get_logger().error('Failed to arm vehicle. Exiting...')
            return
        task_control.ros_sleep(2.0)

        # Takeoff to 1.5 meters
        task_control.get_logger().info('Taking off to 1.5 meters...')
        if not task_control.takeoff(1.5):
            task_control.get_logger().error('Failed to send takeoff command. Landing...')
            task_control.land()
            return
        task_control.get_logger().info('Drone is taking off...')
        task_control.ros_sleep(5.0)  # Wait for takeoff to complete

        '''
        This creates a parametrized Figure 8 waypoint
        '''
        t = np.linspace(0, 2*np.pi, 15)
        x = np.sin(t) * 2
        y = np.sin(2*t)

        waypoints_xy = np.vstack((x, y)).T
        z = np.full((waypoints_xy.shape[0], 1), 1.5)
        waypoints = np.concatenate((waypoints_xy, z), axis=1)

        '''
        This iterates through the waypoints
        '''
        for point in waypoints:
            task_control.get_logger().info(f'moving to point: {point}')
            task_control.goto_xyz(point)

        task_control.get_logger().info('=== Example 1 Complete ===')

    except KeyboardInterrupt:
        task_control.get_logger().info('Flight interrupted by user')
    except Exception as e:
        task_control.get_logger().error(f'An error occurred: {e}')
    finally:
        task_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


        



