"""
landing.py
----------
Methodology 2: Oscillation exploit landing on a moving UGV.

Strategy:
    Phase 1 — POSITION:
        Read /uav/endpoint_aruco_pose (relative to UAV) once.
        Fly to that world-frame XY and hold at Z_HOLD.
        The UGV will oscillate between its origin and this goal point.

    Phase 2 — WAIT:
        Monitor /uav/landing_pad_pose velocity via rolling average.
        Wait for K consecutive ticks of the UGV moving AWAY from goal
        (i.e. returning toward origin). This is the reversal trigger.
        UAV is now ahead of the UGV in its direction of travel.

    Phase 3 — DESCENT:
        UGV is moving toward UAV's position from behind.
        Execute closure-coupled descent: dZ = DESCENT_GAIN * Z * (lateral_speed / lateral_distance)
        UAV sinks, UGV slides underneath, touchdown at near-zero relative velocity.

    Abort gate:
        If lateral error exceeds LATERAL_ABORT_THRESHOLD mid-descent,
        abort back to Phase 1 and re-acquire.

Unit test sequence:
    - Takeoff to Z_HOLD
    - Hover HOVER_DURATION seconds (stability check)
    - Enter Methodology 2

Topics:
    /uav/endpoint_aruco_pose  PoseStamped  UGV oscillation target, relative to UAV
    /uav/landing_pad_pose     PoseStamped  Live pad/UGV position, relative to UAV
    /mavros/local_position/pose  PoseStamped  UAV own position (world frame)

All poses on /uav/endpoint_aruco_pose and /uav/landing_pad_pose are relative to UAV
position and are converted to world frame before use as setpoints.
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
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------
Z_HOLD                   = 2.0    # Hover altitude (m)
HOVER_DURATION           = 5.0    # Seconds to hover before approach
VELOCITY_WINDOW          = 8      # Rolling average buffer size for pad velocity
REVERSAL_TICKS           = 5      # Consecutive ticks of reversed velocity to confirm
DESCENT_GAIN             = 1.0    # Scalar on closure-coupled descent law
MIN_DESCENT_RATE         = 0.05   # Floor descent rate (m/s)
MAX_DESCENT_RATE         = 0.5    # Ceiling descent rate (m/s)
LAND_Z_THRESHOLD         = 0.15   # Altitude below which we call land()
LATERAL_ABORT_THRESHOLD  = 1.5    # Mid-descent lateral error abort limit (m)
GOAL_ARRIVAL_THRESHOLD   = 0.3    # XY threshold to consider goal position reached (m)
LOG_RATE_HZ              = 10     # Logging and setpoint publish rate
SETPOINT_RATE_HZ         = 20     # Setpoint publish rate during hover/prime
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


class OscillationLanding(Node):

    def __init__(self):
        super().__init__('oscillation_landing')

        # ── QoS ──────────────────────────────────────────────────────────────
        mavros_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── State ─────────────────────────────────────────────────────────────
        self.current_pose   = None   # PoseStamped, world frame
        self.pad_pose       = None   # PoseStamped, relative to UAV (live)
        self.goal_pose      = None   # PoseStamped, relative to UAV (latched once)
        self.mission_start  = None   # float, time.time() at arm

        # Goal world-frame XY — computed once from goal_pose + pose at acquisition
        self._goal_world_x  = None
        self._goal_world_y  = None

        # Rolling buffer for pad velocity: deque of (timestamp, x, y)
        self._pad_history   = deque(maxlen=VELOCITY_WINDOW)

        # Reversal detection counter
        self._reversal_count = 0

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
        self.create_subscription(
            PoseStamped,
            '/uav/endpoint_aruco_pose',
            self._goal_callback,
            mavros_qos,
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._sp_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
        )

        # ── Service clients ───────────────────────────────────────────────────
        self._arming_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
        self._land_client    = self.create_client(CommandTOL,  '/mavros/cmd/land')
        self._mode_client    = self.create_client(SetMode,     '/mavros/set_mode')

        for client in (self._arming_client, self._takeoff_client,
                       self._land_client, self._mode_client):
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for {client.srv_name}...')

        # ── Logging ───────────────────────────────────────────────────────────
        self._log_file   = None
        self._csv_writer = None
        self._init_log()

        self.get_logger().info('OscillationLanding node ready.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def _pad_callback(self, msg: PoseStamped):
        self.pad_pose = msg
        t = time.time()
        x = msg.pose.position.x
        y = msg.pose.position.y
        self._pad_history.append((t, x, y))

    def _goal_callback(self, msg: PoseStamped):
        """Latch goal pose once. Convert to world frame on first receipt."""
        if self.goal_pose is not None:
            return  # Already latched
        if self.current_pose is None:
            return  # Cannot convert yet — will retry on next message

        self.goal_pose      = msg
        self._goal_world_x  = self.current_pose.pose.position.x + msg.pose.position.x
        self._goal_world_y  = self.current_pose.pose.position.y + msg.pose.position.y
        self.get_logger().info(
            f'Goal latched: world XY=({self._goal_world_x:.2f}, {self._goal_world_y:.2f})'
        )

    # ── Logging ───────────────────────────────────────────────────────────────

    def _init_log(self):
        stamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
        node_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir  = os.path.join(node_dir, 'logs', stamp)
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, 'oscillation_landing.csv')
        self._log_file   = open(log_path, 'w', newline='')
        self._csv_writer = csv.writer(self._log_file)
        self._csv_writer.writerow(
            ['operation', 'second', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
        )
        self.get_logger().info(f'Logging to {log_path}')

    def _log(self, operation: str):
        if self.current_pose is None or self._csv_writer is None:
            return
        p = self.current_pose.pose.position
        q = self.current_pose.pose.orientation
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
        rate = 1.0 / SETPOINT_RATE_HZ
        for _ in range(count):
            self._publish_setpoint(x, y, z, yaw)
            time.sleep(rate)

    # ── Velocity estimation ───────────────────────────────────────────────────

    def _estimate_pad_velocity(self):
        """
        Rolling average velocity from pad pose history.
        Returns (vx, vy) in pad-relative frame, or (0, 0) if insufficient data.
        """
        if len(self._pad_history) < 2:
            return 0.0, 0.0
        buf = list(self._pad_history)
        dt  = buf[-1][0] - buf[0][0]
        if dt < 1e-6:
            return 0.0, 0.0
        vx = (buf[-1][1] - buf[0][1]) / dt
        vy = (buf[-1][2] - buf[0][2]) / dt
        return vx, vy

    def _is_reversing(self):
        """
        Detect whether the UGV is moving away from goal (returning to origin).

        The goal world position is known. The UGV is reversing when the
        dot product of its velocity vector with the goal-to-origin vector
        is positive — i.e. it is moving away from goal toward origin.

        Returns True if REVERSAL_TICKS consecutive ticks confirm reversal.
        """
        if self._goal_world_x is None or self.current_pose is None:
            return False

        vx, vy = self._estimate_pad_velocity()

        # Vector from goal toward origin (0,0 in world is approximate origin)
        # More precisely: away from goal = negative of (goal - origin) direction
        # Since pad_pose is relative, velocity sign directly encodes direction:
        # When UGV moves away from goal back to origin, relative distance increases.
        # We use dot product of velocity with (origin - goal) direction.
        cp = self.current_pose.pose.position
        gx = self._goal_world_x
        gy = self._goal_world_y

        # Direction from goal to origin (approximate)
        away_x = cp.x - gx  # rough origin direction from goal
        away_y = cp.y - gy
        away_mag = math.hypot(away_x, away_y)
        if away_mag < 1e-3:
            return False

        away_x /= away_mag
        away_y /= away_mag

        dot = vx * away_x + vy * away_y

        if dot > 0:
            self._reversal_count += 1
        else:
            self._reversal_count = 0

        return self._reversal_count >= REVERSAL_TICKS

    # ── Phase logic ───────────────────────────────────────────────────────────

    def hover(self, duration: float):
        """Hold current XY at Z_HOLD for duration seconds."""
        if self.current_pose is None:
            self.get_logger().warn('No pose for hover.')
            return
        p = self.current_pose.pose.position
        q = self.current_pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(f'Hovering {duration}s at z={Z_HOLD}')
        rate = 1.0 / LOG_RATE_HZ
        end  = time.time() + duration
        while time.time() < end:
            self._publish_setpoint(p.x, p.y, Z_HOLD, yaw)
            self._log('HOVER')
            time.sleep(rate)

    def phase_position(self):
        """
        Phase 1: Fly to goal world XY and hold at Z_HOLD.
        Blocks until arrival within GOAL_ARRIVAL_THRESHOLD.
        """
        self.get_logger().info('Phase 1: Positioning above goal.')
        rate = 1.0 / LOG_RATE_HZ
        while True:
            if self.current_pose is None or self._goal_world_x is None:
                time.sleep(rate)
                continue

            cp  = self.current_pose.pose.position
            tx  = self._goal_world_x
            ty  = self._goal_world_y
            yaw = math.atan2(ty - cp.y, tx - cp.x)

            self._publish_setpoint(tx, ty, Z_HOLD, yaw)
            self._log('POSITION')

            dist = math.hypot(tx - cp.x, ty - cp.y)
            self.get_logger().info(f'Phase 1: dist_to_goal={dist:.2f}m')

            if dist <= GOAL_ARRIVAL_THRESHOLD:
                self.get_logger().info('Phase 1 complete — at goal position.')
                break

            time.sleep(rate)

    def phase_wait(self):
        """
        Phase 2: Hold above goal, monitor pad velocity, wait for UGV reversal.
        Blocks until REVERSAL_TICKS consecutive ticks confirm UGV moving away from goal.
        """
        self.get_logger().info('Phase 2: Waiting for UGV reversal.')
        rate = 1.0 / LOG_RATE_HZ
        tx   = self._goal_world_x
        ty   = self._goal_world_y

        while True:
            if self.current_pose is None:
                time.sleep(rate)
                continue

            cp  = self.current_pose.pose.position
            yaw = math.atan2(ty - cp.y, tx - cp.x)
            self._publish_setpoint(tx, ty, Z_HOLD, yaw)
            self._log('WAIT')

            vx, vy = self._estimate_pad_velocity()
            self.get_logger().info(
                f'Phase 2: reversal_count={self._reversal_count}/{REVERSAL_TICKS}  '
                f'vpad=({vx:.2f},{vy:.2f})'
            )

            if self._is_reversing():
                self.get_logger().info('Phase 2 complete — reversal confirmed.')
                break

            time.sleep(rate)

    def phase_descent(self):
        """
        Phase 3: Closure-coupled descent.

        UGV is moving away from goal (toward origin), UAV is above goal.
        UGV approaches from behind — UAV descends, UGV catches up underneath.

        Abort gate: if lateral error exceeds LATERAL_ABORT_THRESHOLD,
        return False so caller can re-enter Phase 1.
        """
        self.get_logger().info('Phase 3: Closure-coupled descent.')
        rate = 1.0 / LOG_RATE_HZ
        dt   = rate

        while True:
            if self.current_pose is None or self.pad_pose is None:
                time.sleep(rate)
                continue

            cp = self.current_pose.pose.position
            cq = self.current_pose.pose.orientation
            _, _, yaw = euler_from_quaternion(cq.x, cq.y, cq.z, cq.w)
            z = cp.z

            # Pad world XY
            pad_wx = cp.x + self.pad_pose.pose.position.x
            pad_wy = cp.y + self.pad_pose.pose.position.y

            # Lateral error from directly above pad
            lat_error = math.hypot(pad_wx - cp.x, pad_wy - cp.y)

            # Abort gate
            if lat_error > LATERAL_ABORT_THRESHOLD:
                self.get_logger().warn(
                    f'Lateral error {lat_error:.2f}m exceeds threshold — aborting descent.'
                )
                return False  # Signal re-acquire

            # Velocity and closure
            vx, vy    = self._estimate_pad_velocity()
            lat_speed = math.hypot(vx, vy)

            if lat_error > 1e-3:
                closure_ratio = lat_speed / lat_error
            else:
                closure_ratio = 1.0

            raw_dz = DESCENT_GAIN * z * closure_ratio
            dz     = max(MIN_DESCENT_RATE, min(MAX_DESCENT_RATE, raw_dz))
            new_z  = max(0.0, z - dz * dt)

            target_yaw = math.atan2(pad_wy - cp.y, pad_wx - cp.x)
            self._publish_setpoint(pad_wx, pad_wy, new_z, target_yaw)
            self._log('DESCENT')

            self.get_logger().info(
                f'Phase 3: lat_err={lat_error:.2f}m  z={new_z:.2f}m  '
                f'dz={dz:.3f}m/s  vpad=({vx:.2f},{vy:.2f})'
            )

            if new_z <= LAND_Z_THRESHOLD:
                self.get_logger().info('Threshold reached — calling land().')
                self.land()
                return True  # Complete

            time.sleep(rate)


def main(args=None):
    rclpy.init(args=args)
    node = OscillationLanding()

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
        time.sleep(3.0)

        # Unit test: hover stability check
        node.hover(HOVER_DURATION)

        # Wait for goal pose to be latched
        node.get_logger().info('Waiting for goal pose...')
        while node._goal_world_x is None:
            time.sleep(0.1)

        # Wait for pad pose
        node.get_logger().info('Waiting for pad pose...')
        while node.pad_pose is None:
            time.sleep(0.1)

        # Main landing loop with abort/re-acquire
        while True:
            node.phase_position()
            node.phase_wait()
            landed = node.phase_descent()
            if landed:
                break
            node.get_logger().info('Re-acquiring — restarting from Phase 1.')

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node._close_log()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()