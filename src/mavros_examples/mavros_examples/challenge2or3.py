#!/usr/bin/env python3
import rclpy  # noqa: I100
from geometry_msgs.msg import PoseStamped,TwistStamped,Point  # noqa: F401,I100
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
        self.ugv_loc_x = None
        self.ugv_loc_y = None
        self.goal_reached = False
        self.target_found = False
        self.ugv_marker_found = False

        self.ugv_loc_sub = self.create_subscription(
            Point, "/uav/peer/ugv_location", self.ugv_loc_callback, 3)

        self.goal_reached_sub = self.create_subscription(
            Bool, "/goal_reached", self.goal_callback, 3)

        self.target_sub = self.create_subscription(
            PoseStamped, "/ugv/goal_pose", self.target_callback, 3)

        self.ugv_marker_sub = self.create_subscription(
            PoseStamped, "/uav/landing_pad_pose", self.ugv_marker_callback, 3)

        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/vision_pose/pose", self.pose_callback, 3)

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

        self.ugv_loc_x = msg.x 
        self.ugv_loc_y = msg.y

    def target_callback(self, msg):

        if not self.target_found:
            self.get_logger().info('Target pose received')
            self.target_found = True

    def ugv_marker_callback(self, msg):

        if not self.ugv_marker_found:
            self.get_logger().info('UGV marker pose received')
            self.ugv_marker_found = True

    

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
        """Return the next step toward the current UGV position."""
        if self.ugv_loc_x is None or self.ugv_loc_y is None:
            self.get_logger().warn('Cannot compute next UGV step without cached UGV location')
            return None

        dx = self.ugv_loc_x - self.current_pose.pose.position.x
        dy = self.ugv_loc_y - self.current_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(f'My Position x:{self.current_pose.pose.position.x}, y:{self.current_pose.pose.position.y}')

        if distance == 0.0:
            return 0.0, 0.0

        if distance <= step_size:
            return self.ugv_loc_x, self.ugv_loc_y

        scale = step_size / distance
        return self.ugv_loc_x * scale, self.ugv_loc_y * scale

def main(args=None):
    rclpy.init(args=args)

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
        '''search_waypoints = [
            ( 1.0, 0.0, 1.5),
            (0.0, 0.0, 1.5),
            ( -1.0, 0.0, 1.5),
            ( 0.0, 0.0, 1.5),
            ( 0.0, 1.0, 1.5),
            ( 0.0, 0.0, 1.5),
            ( 0.0, -1.0, 1.5),
        ]'''
        

        for (x, y, z) in search_waypoints:
            
            task_control.get_logger().info(f'Going to point {x}, {y}, {z}')
            task_control.goto_pose(x,y,z,0.0)
            if task_control.target_found: 
                break

        task_control.get_logger().info(f'Going to UGV Position')
        hover_height = 1.5        

        while not task_control.ugv_marker_found:

            dx, dy = task_control.waypoints_to_ugv()
            task_control.get_logger().info(f'Going to point {dx}, {dy}, {hover_height}')
            task_control.goto_pose(dx, dy, hover_height, 0.0)
        
        #Follows ArUco


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


        
