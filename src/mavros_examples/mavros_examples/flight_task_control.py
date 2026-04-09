import rclpy  # noqa: I100
from geometry_msgs.msg import PoseStamped  # noqa: F401,I100
from mavros_msgs.msg import State  # noqa: F401
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode
from rclpy.node import Node


class TaskControl(Node):
    """Control node."""

    def __init__(self):
        super().__init__('task_control')

        self.cmd_pos_pub = self.create_publisher( PoseStamped,
            "/mavros/setpoint_position/local",10)

        self.pose_sub = self.create_subscriber(PoseStamped,"/mavros/local_position/pose", self.pose_callback, 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')

        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.wait_for_services()
        self.get_logger().info('MAVROS services are ready!')

    def wait_for_services(self):
        """Wait for all services to be available."""
        services = [
            self.arming_client,
            self.set_mode_client,
            self.takeoff_client,
            self.land_client,
            self.set_home_client,
        ]

        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service...')

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose


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


    def goto(self, pose: Pose):
        """
        Publish a position setpoint. Vehicle should be in GUIDED mode.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = pose
        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, roll, pitch, yaw):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        self.goto(pose)



def main(args=None):
    rclpy.init(args=args)

    try:
        # Create TaskControl instance
        task_control = TaskControl()

        task_control.get_logger().info('=== Example 1: Basic Flight Sequence ===')
        import time

        time.sleep(2)

        # Set to GUIDED mode
        task_control.get_logger().info('Setting mode to GUIDED...')
        if not task_control.set_mode('GUIDED'):
            task_control.get_logger().error('Failed to set GUIDED mode. Exiting...')
            return
        time.sleep(2)

        # Arm the vehicle
        task_control.get_logger().info('Arming the vehicle...')
        if not task_control.arm():
            task_control.get_logger().error('Failed to arm vehicle. Exiting...')
            return
        time.sleep(2)

        # Takeoff to 2 meters
        task_control.get_logger().info('Taking off to 2 meters...')
        if not task_control.takeoff(2.0):
            task_control.get_logger().error('Failed to send takeoff command. Landing...')
            task_control.land()
            return
        task_control.get_logger().info('Drone is taking off...')
        time.sleep(15)  # Wait for takeoff to complete

        task_control.get_logger().info('Executing position sequence...')
        altitude = 2.0
        move_hold_time = 5.0
        roll = 0.0
        pitch = 0.0
        yaw= 0.0

        waypoints = [
            ('front', 1.0, 0.0, altitude, move_hold_time),
            ('origin', 0.0, 0.0, altitude, settle_hold_time),
            ('back', -1.0, 0.0, altitude, move_hold_time),
            ('origin', 0.0, 0.0, altitude, settle_hold_time),
            ('left', 0.0, 1.0, altitude, move_hold_time),
            ('origin', 0.0, 0.0, altitude, settle_hold_time),
            ('right', 0.0, -1.0, altitude, move_hold_time),
        ]

        for name, x, y, z, hold_time in waypoints:
            task_control.get_logger().info(
                f'Moving {name}: x={x:.1f}, y={y:.1f}, z={z:.1f}'
            )
            task_control.goto_xyz_rpy(x,y,z,0,0,0)
            time.sleep(5)



        task_control.get_logger().info('Landing...')
        if not task_control.land():
            task_control.get_logger().error('Failed to send land, land manually')
            return

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
