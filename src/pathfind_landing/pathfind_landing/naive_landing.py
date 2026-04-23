"""
naive_landing.py
----------------
Methodology 1: Naive landing on a moving UGV.

Strategy:
    - Fly directly to pad XY from /uav/landing_pad_pose (relative to UAV)
    - Execute closure-coupled descent: dZ = -Z * (lateral_speed / lateral_distance)
    - No heading alignment gate
    - ArUco correction stubbed for future integration

Unit test sequence:
    - Takeoff to Z_HOLD
    - Hover HOVER_DURATION seconds
    - Enter naive landing approach

Topics:
    /uav/landing_pad_pose   PoseStamped  UGV/pad position, relative to UAV
    /mavros/local_position/pose  PoseStamped  UAV own position (world frame)

All pad poses are relative to UAV position and must be added to current
UAV world pose before use as setpoints.
"""

import os
import csv
import math
import time
import threading
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------
Z_HOLD              = 2.0       # Hover altitude (m)
HOVER_DURATION      = 5.0       # Seconds to hover before approach
VELOCITY_WINDOW     = 8         # Rolling average buffer size for pad velocity
DESCENT_GAIN        = 1.0       # Scalar on closure-coupled descent law
MIN_DESCENT_RATE    = 0.05      # Floor descent rate (m/s) so drone never stalls
MAX_DESCENT_RATE    = 0.5       # Ceiling descent rate (m/s)
LAND_Z_THRESHOLD    = 0.15      # Altitude below which we call land()
LATERAL_THRESHOLD   = 0.2       # XY arrival threshold (m)
LOG_RATE_HZ         = 10        # Logging and setpoint publish rate
SETPOINT_RATE_HZ    = 20        # Setpoint publish rate during hover/prime
ARRIVAL_YAW_DEG     = 5.0       # Yaw threshold for arrival check (degrees)
# ---------------------------------------------------------------------------


def euler_from_quaternion(x, y, z, w):
    """Return (roll, pitch, yaw) in radians from quaternion."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_yaw(yaw):
    """Return (x, y, z, w) quaternion for a pure yaw rotation."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NaiveLanding(Node):

    def __init__(self):
        super().__init__('naive_landing')

        # ── QoS ──────────────────────────────────────────────────────────────
        mavros_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── State ─────────────────────────────────────────────────────────────
        self.current_pose   = None   # PoseStamped, world frame
        self.pad_pose       = None   # PoseStamped, relative to UAV
        self.mission_start  = None   # float, time.time() at arm

        # Rolling buffer for pad velocity estimation: deque of (timestamp, x, y)
        self._pad_history = deque(maxlen=VELOCITY_WINDOW)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            mavros_qos,
        )
        self.create_subscription(
            PoseStamped,
            '/uav/landing_pad_pose',
            self._pad_callback,
            mavros_qos,
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._sp_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
        )

        # ── Service clients ───────────────────────────────────────────────────
        self._arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_client  = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
        self._land_client     = self.create_client(CommandTOL,  '/mavros/cmd/land')
        self._mode_client     = self.create_client(SetMode,     '/mavros/set_mode')

        for client in (self._arming_client, self._takeoff_client,
                       self._land_client, self._mode_client):
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for {client.srv_name}...')

        # ── Logging ───────────────────────────────────────────────────────────
        self._log_file   = None
        self._csv_writer = None
        self._init_log()

        self.get_logger().info('NaiveLanding node ready.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def _pad_callback(self, msg: PoseStamped):
        self.pad_pose = msg
        t = time.time()
        x = msg.pose.position.x
        y = msg.pose.position.y
        self._pad_history.append((t, x, y))

    # ── Logging ───────────────────────────────────────────────────────────────

    def _init_log(self):
        stamp     = datetime.now().strftime('%Y%m%d_%H%M%S')
        node_dir  = os.path.dirname(os.path.abspath(__file__))
        log_dir   = os.path.join(node_dir, 'logs', stamp)
        os.makedirs(log_dir, exist_ok=True)
        log_path  = os.path.join(log_dir, 'naive_landing.csv')
        self._log_file   = open(log_path, 'w', newline='')
        self._csv_writer = csv.writer(self._log_file)
        self._csv_writer.writerow(
            ['operation', 'second', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
        )
        self.get_logger().info(f'Logging to {log_path}')

    def _log(self, operation: str):
        if self.current_pose is None or self._csv_writer is None:
            return
        p   = self.current_pose.pose.position
        q   = self.current_pose.pose.orientation
        r, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        elapsed = round(time.time() - self.mission_start, 1) if self.mission_start else 0.0
        self._csv_writer.writerow([operation, elapsed, p.x, p.y, p.z, r, pitch, yaw])

    def _close_log(self):
        if self._log_file:
            self._log_file.close()

    # ── MAVROS primitives ─────────────────────────────────────────────────────

    def set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        success = future.result() and future.result().mode_sent
        self.get_logger().info(f'set_mode({mode}): {"OK" if success else "FAILED"}')
        return success

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self._arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        success = future.result() and future.result().success
        self.get_logger().info(f'arm(): {"OK" if success else "FAILED"}')
        return success

    def takeoff(self, altitude: float):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self._takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        success = future.result() and future.result().success
        self.get_logger().info(f'takeoff({altitude}m): {"OK" if success else "FAILED"}')
        return success

    def land(self):
        req = CommandTOL.Request()
        future = self._land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        success = future.result() and future.result().success
        self.get_logger().info(f'land(): {"OK" if success else "FAILED"}')
        return success

    def _publish_setpoint(self, x, y, z, yaw):
        """Publish a world-frame position setpoint."""
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        qx, qy, qz, qw     = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._sp_pub.publish(msg)

    def _prime_setpoints(self, x, y, z, yaw, count=30):
        """Pump setpoints before arming to satisfy FCU."""
        rate = 1.0 / SETPOINT_RATE_HZ
        for _ in range(count):
            self._publish_setpoint(x, y, z, yaw)
            time.sleep(rate)

    # ── Velocity estimation ───────────────────────────────────────────────────

    def _estimate_pad_velocity(self):
        """
        Rolling average velocity estimate from pad pose history.
        Returns (vx, vy) in pad-relative frame, or (0, 0) if insufficient data.
        """
        if len(self._pad_history) < 2:
            return 0.0, 0.0

        buf  = list(self._pad_history)
        dt   = buf[-1][0] - buf[0][0]
        if dt < 1e-6:
            return 0.0, 0.0

        vx = (buf[-1][1] - buf[0][1]) / dt
        vy = (buf[-1][2] - buf[0][2]) / dt
        return vx, vy

    # ── ArUco stub ────────────────────────────────────────────────────────────

    def _aruco_correction(self):
        """
        Stub for ArUco marker correction.
        Returns (dx, dy) offset correction in world frame.
        Replace with real marker pose lookup when available.
        """
        return 0.0, 0.0

    # ── Core landing logic ────────────────────────────────────────────────────

    def _pad_world_xy(self):
        """
        Convert relative pad pose to world frame XY.
        pad_pose is relative to UAV position.
        """
        if self.current_pose is None or self.pad_pose is None:
            return None, None
        wx = self.current_pose.pose.position.x + self.pad_pose.pose.position.x
        wy = self.current_pose.pose.position.y + self.pad_pose.pose.position.y
        return wx, wy

    def hover(self, duration: float):
        """Hold current XY position at Z_HOLD for duration seconds."""
        if self.current_pose is None:
            self.get_logger().warn('No pose available for hover.')
            return

        p   = self.current_pose.pose.position
        q   = self.current_pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        self.get_logger().info(f'Hovering for {duration}s at z={Z_HOLD}')
        rate    = 1.0 / LOG_RATE_HZ
        end     = time.time() + duration
        while time.time() < end:
            self._publish_setpoint(p.x, p.y, Z_HOLD, yaw)
            self._log('HOVER')
            time.sleep(rate)

    def naive_approach(self):
        """
        Naive closure-coupled descent toward pad.

        Each tick:
          1. Compute world-frame pad XY (relative pose + own pose)
          2. Apply ArUco correction (stub)
          3. Compute lateral distance and speed
          4. Compute descent rate: dZ = DESCENT_GAIN * Z * (lateral_speed / lateral_distance)
          5. Publish setpoint at (pad_x, pad_y, Z - dZ*dt)
          6. Log tick
        """
        if self.current_pose is None:
            self.get_logger().warn('No pose — cannot begin approach.')
            return

        self.get_logger().info('Beginning naive approach.')
        rate = 1.0 / LOG_RATE_HZ
        dt   = rate

        while True:
            if self.current_pose is None or self.pad_pose is None:
                time.sleep(rate)
                continue

            # Current UAV state
            cp  = self.current_pose.pose.position
            cq  = self.current_pose.pose.orientation
            _, _, yaw = euler_from_quaternion(cq.x, cq.y, cq.z, cq.w)
            z   = cp.z

            # Target XY in world frame
            tx, ty = self._pad_world_xy()
            if tx is None:
                time.sleep(rate)
                continue

            # ArUco correction
            dx, dy = self._aruco_correction()
            tx += dx
            ty += dy

            # Lateral distance and speed
            lat_dist  = math.hypot(tx - cp.x, ty - cp.y)
            vx, vy    = self._estimate_pad_velocity()
            lat_speed = math.hypot(vx, vy)

            # Closure-coupled descent rate
            if lat_dist > 1e-3:
                closure_ratio = lat_speed / lat_dist
            else:
                closure_ratio = 1.0  # Already over pad — descend at full gain

            raw_dz    = DESCENT_GAIN * z * closure_ratio
            dz        = max(MIN_DESCENT_RATE, min(MAX_DESCENT_RATE, raw_dz))
            new_z     = max(0.0, z - dz * dt)

            # Yaw toward pad
            target_yaw = math.atan2(ty - cp.y, tx - cp.x)

            self._publish_setpoint(tx, ty, new_z, target_yaw)
            self._log('APPROACH')

            self.get_logger().info(
                f'lat_dist={lat_dist:.2f}m  z={new_z:.2f}m  dz={dz:.3f}m/s  '
                f'vpad=({vx:.2f},{vy:.2f})'
            )

            # Terminal condition
            if new_z <= LAND_Z_THRESHOLD:
                self.get_logger().info('Threshold reached — calling land().')
                self.land()
                break

            time.sleep(rate)


def main(args=None):
    rclpy.init(args=args)
    node = NaiveLanding()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Wait for pose
        node.get_logger().info('Waiting for pose...')
        while node.current_pose is None:
            time.sleep(0.1)

        node.mission_start = time.time()

        # Prime setpoints
        p = node.current_pose.pose.position
        node._prime_setpoints(p.x, p.y, Z_HOLD, 0.0)

        # Arm and takeoff
        node.set_mode('GUIDED')
        node.arm()
        node.takeoff(Z_HOLD)
        time.sleep(3.0)  # Allow FCU to climb

        # Unit test: hover phase
        node.hover(HOVER_DURATION)

        # Wait for pad pose before approach
        node.get_logger().info('Waiting for pad pose...')
        while node.pad_pose is None:
            time.sleep(0.1)

        # Naive landing approach
        node.naive_approach()

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node._close_log()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()