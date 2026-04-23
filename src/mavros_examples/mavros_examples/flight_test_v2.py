#!/usr/bin/env python3
import rclpy  # noqa: I100
from geometry_msgs.msg import PoseStamped,TwistStamped  # noqa: F401,I100
from mavros_msgs.msg import State  # noqa: F401
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from std_msgs.msg import Bool

import time
import math


class TaskControl(Node):
    def __init__(self):
        super().__init__('task_control')

        self.initial_pose = None
        self.current_pose = None
        self.goal_reached = False

        self.goal_reached_sub = self.create_subscription(
            Bool, "/goal_reached", self.goal_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/vision_pose/pose", self.pose_callback, 5)

        self.goal_pose_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 5)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

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

        self.current_pose = pose_msg

        if self.initial_pose is None:
            self.initial_pose = pose_msg

    def goal_callback(self, goal_msg):

        self.get_logger().info(f'Goal Reached message received: {goal_msg}')

        self.goal_reached = goal_msg.data

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

    def ros_sleep(self, duration_sec):

        end = time.monotonic() + duration_sec

        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_initial_pose(self, timeout_sec=5.0):
        start = time.monotonic()

        while rclpy.ok() and self.initial_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.monotonic() - start > timeout_sec:
                return False

        return True

    def goto_pose(self, x, y, z, yaw):

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x + self.initial_pose.pose.position.x
        goal.pose.position.y = y + self.initial_pose.pose.position.y
        goal.pose.position.z = z + self.initial_pose.pose.position.z

        goal.pose.orientation.z = yaw + self.initial_pose.pose.orientation.z
        self.goal_reached = False

        self.goal_pose_pub.publish(goal)

        while rclpy.ok() and not self.goal_reached:

            rclpy.spin_once(self, timeout_sec=0.1)

    # This function checks if the current pose matches the value given
    # def is_at_pose(self, x, y, z, tol=0.2):
    #         dx = self.current_pose.position.x - x
    #         dy = self.current_pose.position.y - y
    #         dz = self.current_pose.position.z - z

    #         return (dx*dx + dy*dy + dz*dz) ** 0.5 < tol
    # This function generates waypoints in a figure 8 pattern, waypoint number can be changed to be much more accurate pattern
    def generate_parallel_waypoints(self, start_x, start_y, height,
                                 width=4.0, height_y=4.0, step=1.0):
            """
            width: total distance in X direction
            height_y: total distance in Y direction
            step: spacing between each sweep line
            """
            waypoints = []

            num_lines = int(height_y / step)
            direction = 1  # 1 = forward, -1 = backward

            # For loop to execute parallel line search
            for i in range(num_lines + 1):
                y = start_y + i * step

                if direction == 1:
                    # left → right
                    waypoints.append((start_x, y, height))
                    waypoints.append((start_x + width, y, height))
                else:
                    # right → left
                    waypoints.append((start_x + width, y, height))
                    waypoints.append((start_x, y, height))

                direction *= -1  # flip direction each row

            return waypoints


def main(args=None):
    rclpy.init(args=args)

    try:
        task_control = TaskControl()

        task_control.get_logger().info('=== Flight Test V2: Takeoff, Move 2m, Land ===')

        task_control.get_logger().info('Waiting for initial Vicon pose')
        if not task_control.wait_for_initial_pose(timeout_sec=5.0):
            task_control.get_logger().error('Did not receive initial pose. Exiting without takeoff.')
            return

        # Set to GUIDED mode
        task_control.get_logger().info('Setting mode to GUIDED...')
        if not task_control.set_mode('GUIDED'):
            task_control.get_logger().error('Failed to set GUIDED mode. Exiting...')
            return
        task_control.ros_sleep(1.0)

        # Arm the vehicle
        task_control.get_logger().info('Arming the vehicle...')
        if not task_control.arm():
            task_control.get_logger().error('Failed to arm vehicle. Exiting...')
            return
        task_control.ros_sleep(1.0)

        # Takeoff to 1.5 meters
        task_control.get_logger().info('Taking off to 1.5 meters...')
        if not task_control.takeoff(1.5):
            task_control.get_logger().error('Failed to send takeoff command. Landing...')
            task_control.land()
            return
        task_control.get_logger().info('Drone is taking off...')
        task_control.ros_sleep(5.0)  # Wait for takeoff to complete

        # Sleep for 2.5915 seconds after takeoff
        task_control.get_logger().info('Sleeping for 2.5915 seconds...')
        task_control.ros_sleep(2.5915)

        # Move 2 meters in +X direction (the original file's first travel direction)
        task_control.get_logger().info('Going to point 2.0, 0.0, 1.5')
        task_control.goto_pose(2.0, 0.0, 1.5, 0)

        # Sleep for 2 seconds
        task_control.get_logger().info('Sleeping for 2 seconds...')
        task_control.ros_sleep(2.0)

        # Land
        task_control.get_logger().info('Landing...')
        if not task_control.land():
            task_control.get_logger().error('Failed to send land, land manually')
            return


    except KeyboardInterrupt:
        task_control.get_logger().info('Flight interrupted by user')
    except Exception as e:
        task_control.get_logger().error(f'An error occurred: {e}')
    finally:
        task_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
