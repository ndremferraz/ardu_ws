#!/usr/bin/env python3
import rclpy  # noqa: I100
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: F401,I100
from mavros_msgs.msg import State  # noqa: F401
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from std_msgs.msg import Bool, String

import time
import math

class TaskControl(Node):
    def __init__(self):
        super().__init__('task_control')

        self.initial_pose = None
        self.current_pose = None

        self.goal_reached = False

        self.target_found = False
        self.pad_found = False

        self.ugv_loc = None
        self.pad_loc = None

        self.ugv_loc_sub = self.create_subscription(
            PoseStamped, "/uav/peer/ugv_location", self.ugv_loc_callback, 3)

        self.goal_reached_sub = self.create_subscription(
            Bool, "/goal_reached", self.goal_callback, 3)

        self.target_sub = self.create_subscription(
            PoseStamped, "/target_aruco_pose", self.target_callback, 3)

        self.pad_sub = self.create_subscription(
            PoseStamped, "/pad_aruco_pose", self.pad_callback, 3)

        self.pose_sub = self.create_subscription(
            PoseStamped, "/zed/zed_node/pose", self.pose_callback, 3)

        self.goal_pose_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 3)

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

    def ugv_loc_callback(self, msg):

        self.ugv_loc = msg 

    def target_callback(self, msg):

        if not self.target_found:
            self.get_logger().info('Target pose received')
            self.target_found = True

    def pad_callback(self, msg):

        if not self.pad_found:
            self.get_logger().info('Pad pose received')
            self.pad_found = True

        self.pad_loc = msg


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

    def waypoints_to_ugv(self, step_size=1.0):
        """Return the next x/y waypoint toward the current UGV pose."""
        if self.ugv_loc is None or self.current_pose is None or self.initial_pose is None:
            self.get_logger().warn('Cannot compute next UGV step without cached UGV location')

            if self.ugv_loc is None:
                self.get_logger().warn('UGV loc in None')

            if self.current_pose is None:
                self.get_logger().warn('Current Pose is None')

            if self.initial_pose is None:
                self.get_logger().warn('Initial Pose is None')
            
            return None

        current_x = self.current_pose.pose.position.x - self.initial_pose.pose.position.x
        current_y = self.current_pose.pose.position.y - self.initial_pose.pose.position.y
        target_x = self.ugv_loc.pose.position.x
        target_y = self.ugv_loc.pose.position.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        self.get_logger().info(f'My Position x:{current_x}, y:{current_y}')

        if distance == 0.0:
            return current_x, current_y

        if distance <= step_size:
            return target_x, target_y

        scale = step_size / distance
        return current_x + (dx * scale), current_y + (dy * scale)

    def waypoints_to_pad(self, step_size=1.5):
        """Return the next x/y/z waypoint toward the current pad pose."""
        if self.pad_loc is None or self.current_pose is None or self.initial_pose is None:
            self.get_logger().warn('Cannot compute next pad step without cached pad location')
            return None

        current_x = self.current_pose.pose.position.x - self.initial_pose.pose.position.x
        current_y = self.current_pose.pose.position.y - self.initial_pose.pose.position.y
        current_z = self.current_pose.pose.position.z - self.initial_pose.pose.position.z
        target_x = self.pad_loc.pose.position.x
        target_y = self.pad_loc.pose.position.y
        target_z = self.pad_loc.pose.position.z

        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        self.get_logger().info(f'My Position x:{current_x}, y:{current_y}, z:{current_z}')

        if distance == 0.0:
            return current_x, current_y, current_z

        if distance <= step_size:
            return target_x, target_y, target_z

        scale = step_size / distance
        return (
            current_x + (dx * scale),
            current_y + (dy * scale),
            current_z + (dz * scale),
        )

    def ready_to_land(self, xy_tolerance=0.2, z_tolerance=0.15):
        """Return True when the vehicle is close enough to the cached pad pose."""
        if self.pad_loc is None or self.current_pose is None or self.initial_pose is None:
            self.get_logger().warn('Cannot evaluate landing readiness without current pose and pad pose')
            return False

        current_x = self.current_pose.pose.position.x - self.initial_pose.pose.position.x
        current_y = self.current_pose.pose.position.y - self.initial_pose.pose.position.y
        current_z = self.current_pose.pose.position.z - self.initial_pose.pose.position.z
        pad_x = self.pad_loc.pose.position.x
        pad_y = self.pad_loc.pose.position.y
        pad_z = self.pad_loc.pose.position.z

        xy_error = math.hypot(pad_x - current_x, pad_y - current_y)
        z_error = abs(pad_z - current_z)
        ready = xy_error <= xy_tolerance and z_error <= z_tolerance

        self.get_logger().info(
            f'Landing check xy_error={xy_error:.2f}, z_error={z_error:.2f}, ready={ready}'
        )
        return ready

def main(args=None):
    rclpy.init(args=args)
    task_control = None

    try:
        task_control = TaskControl()

        task_control.get_logger().info('=== Example 1: Basic Flight Sequence ===')

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

        # Executing Moving in pattern determined by waypoints until marker is found 
        new_waypoints = [
            # --- Approach: (0,0) → center (6.8,6.8) in 1m increments ---
            (0.00, 0.00, 2.3),
            (0.71, 0.71, 2.3),
            (1.41, 1.41, 2.3),
            (2.12, 2.12, 2.3),
            (2.83, 2.83, 2.3),
            (3.54, 3.54, 2.3),
            (4.24, 4.24, 2.3),
            (4.95, 4.95, 2.3),
            (5.66, 5.66, 2.3),
            (6.36, 6.36, 2.3),
            (6.80, 6.80, 2.3),  # Center
            # --- Spiral outward ---
            (6.86, 6.82, 2.3),
            (6.89, 6.89, 2.3),
            (6.87, 6.97, 2.3),
            (6.80, 7.05, 2.3),
            (6.68, 7.09, 2.3),
            (6.53, 7.07, 2.3),
            (6.40, 6.97, 2.3),
            (6.30, 6.80, 2.3),
            (6.28, 6.58, 2.3),
            (6.36, 6.36, 2.3),
            (6.54, 6.16, 2.3),
            (6.80, 6.05, 2.3),
            (7.11, 6.05, 2.3),
            (7.42, 6.18, 2.3),
            (7.67, 6.44, 2.3),
            (7.80, 6.80, 2.3),
            (7.78, 7.21, 2.3),
            (7.60, 7.60, 2.3),
            (7.25, 7.90, 2.3),
            (6.80, 8.05, 2.3),
            (6.30, 8.01, 2.3),
            (5.83, 7.77, 2.3),
            (5.47, 7.35, 2.3),
            (5.30, 6.80, 2.3),
            (5.36, 6.20, 2.3),
            (5.65, 5.65, 2.3),
            (6.15, 5.24, 2.3),
            (6.80, 5.05, 2.3),
            (7.49, 5.13, 2.3),
            (8.13, 5.47, 2.3),
            (8.59, 6.06, 2.3),
            (8.80, 6.80, 2.3),
            (8.71, 7.59, 2.3),
            (8.30, 8.30, 2.3),
            (7.64, 8.82, 2.3),
            (6.80, 9.05, 2.3),
            (5.92, 8.94, 2.3),
            (5.12, 8.48, 2.3),
            (4.55, 7.73, 2.3),
            (4.30, 6.80, 2.3),
            (4.43, 5.82, 2.3),
            (4.94, 4.94, 2.3),
            (5.77, 4.32, 2.3),
            (6.80, 4.05, 2.3),
            (7.88, 4.20, 2.3),
            (8.83, 4.77, 2.3),
            (9.51, 5.68, 2.3),
            (9.80, 6.80, 2.3),
            (9.63, 7.97, 2.3),
            (9.01, 9.01, 2.3),
            (8.02, 9.74, 2.3),
            (6.80, 10.05, 2.3),
            (5.53, 9.86, 2.3),
            (4.41, 9.19, 2.3),
            (3.62, 8.12, 2.3),
            (3.30, 6.80, 2.3),
            (3.51, 5.44, 2.3),
            (4.24, 4.24, 2.3),
            (5.39, 3.39, 2.3),
            (6.80, 3.05, 2.3),
            (8.26, 3.28, 2.3),
            (9.54, 4.06, 2.3),
            (10.44, 5.29, 2.3),
            (10.80, 6.80, 2.3),
            (10.55, 8.35, 2.3),
            (9.72, 9.72, 2.3),
            (8.40, 10.67, 2.3),
            (6.80, 11.05, 2.3),
            (5.15, 10.78, 2.3),
            (3.71, 9.89, 2.3),
            (2.70, 8.50, 2.3),
            (2.30, 6.80, 2.3),
            (2.58, 5.05, 2.3),
            (3.53, 3.53, 2.3),
            (5.01, 2.47, 2.3),
            (6.80, 2.05, 2.3),
            (8.64, 2.35, 2.3),
            (10.25, 3.35, 2.3),
            (11.36, 4.91, 2.3),
            (11.80, 6.80, 2.3),
            (11.48, 8.74, 2.3),
            (10.42, 10.42, 2.3),
            (8.79, 11.59, 2.3),
            (6.80, 12.05, 2.3),
            (4.77, 11.71, 2.3),
            (3.00, 10.60, 2.3),
            (1.78, 8.88, 2.3),
            (1.30, 6.80, 2.3),
            (1.66, 4.67, 2.3),
            (2.82, 2.82, 2.3),
            (4.62, 1.55, 2.3),
            (6.80, 1.05, 2.3),
            (9.02, 1.43, 2.3),
            (10.95, 2.65, 2.3),
            (12.29, 4.53, 2.3),
            (12.80, 6.80, 2.3),
        ]


        for (x, y, z) in search_waypoints:

            task_control.get_logger().info(f'Going to point {x}, {y}, {z}')
            task_control.goto_pose(x,y,z,0.0)
            if task_control.target_found: 
                break

        task_control.get_logger().info(f'Going to UGV Position')
        hover_height = 1.5        

        while not task_control.pad_found:

            waypoint = task_control.waypoints_to_ugv()
            if waypoint is None:
                rclpy.spin_once(task_control, timeout_sec=0.1)
                continue

            dx, dy = waypoint
            task_control.get_logger().info(f'Going to point {dx}, {dy}, {hover_height}')
            task_control.goto_pose(dx, dy, hover_height, 0.0)

        while not task_control.ready_to_land():

            waypoint = task_control.waypoints_to_pad()
            if waypoint is None:
                rclpy.spin_once(task_control, timeout_sec=0.1)
                continue

            dx, dy, dz = waypoint
            task_control.get_logger().info(f'Going to point {dx}, {dy}, {dz}')
            task_control.goto_pose(dx, dy, dz, 0.0)


        task_control.get_logger().info('Landing...')
        if not task_control.land():
            task_control.get_logger().error('Failed to send land, land manually')
            return


    except KeyboardInterrupt:
        task_control.get_logger().info('Flight interrupted by user')
    except Exception as e:
        task_control.get_logger().error(f'An error occurred: {e}')
    finally:
        if task_control is not None:
            task_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


