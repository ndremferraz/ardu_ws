#!/usr/bin/env python3
"""
hybrid_landing.py
-----------------
Methodology 3: Hybrid shadow-then-descend landing on a moving UGV.

Strategy:
    Phase 1 — SHADOW:
        Track /uav/bottom/pad_aruco_pose each tick, publishing the pad's world-frame
        XY at float_height for pursuit_seconds. UAV implicitly matches UGV
        velocity by continuously updating its setpoint each tick.

    Phase 2 — DESCENT:
        After pursuit_seconds elapses, begin closure-coupled descent each tick:
            dZ = DESCENT_GAIN * target_z * (lateral_speed / lateral_distance)
        target_z accumulates each tick — drone descends continuously regardless
        of lateral closure ratio, floored at MIN_DESCENT_RATE.

Control pattern:
    Near-continuous publish at TICK_RATE_HZ. Each tick publishes one setpoint
    and calls spin_once before the next. No goal_reached gating.

Topics:
    /uav/bottom/pad_aruco_pose   PoseStamped  Live pad/UGV position
    /mavros/vision_pose/pose     PoseStamped  UAV own position (world frame)
    /goal_pose                   PoseStamped  Setpoint output

Flags:
    PAD_POSE_RELATIVE  If True, pad pose is relative to UAV and initial_pose
                       offset is applied. If False, pad pose is absolute.
"""

import math
import os
import csv
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------
FLOAT_HEIGHT            = 1.5    # Altitude to shadow UGV during pursuit (m)
PURSUIT_SECONDS         = 5.0    # Duration to shadow before descent begins (s)
HOVER_DURATION          = 1.0    # Stability hover before sequence begins (s)
TICK_RATE_HZ            = 10     # Setpoint publish rate for all phases
VELOCITY_WINDOW         = 8      # Rolling average buffer size for pad velocity
DESCENT_GAIN            = 1.0    # Scalar on closure-coupled descent law
MIN_DESCENT_RATE        = 0.5    # Floor descent rate (m/s)
XY_SPEED                = 0.5    # Max XY step per tick (m/s) — prevents oscillation
MAX_DESCENT_RATE        = 1.5    # Ceiling descent rate (m/s)
LAND_Z_THRESHOLD        = 0.15   # Altitude below which we call land()

# Toggle once pad pose frame is confirmed empirically
PAD_POSE_RELATIVE       = False  # True = pad pose relative to UAV/initial_pose
# ---------------------------------------------------------------------------


class HybridLanding(Node):

    def __init__(self):
        super().__init__('hybrid_landing')

        # ── State ─────────────────────────────────────────────────────────────
        self.initial_pose  = None   # PoseStamped, latched on first pose message
        self.current_pose  = None   # PoseStamped, updated every pose message
        self.pad_pose      = None   # PoseStamped, latest pad position
        self.mission_start = None   # float, time.time() at mission entry

        # Rolling buffer for pad velocity: deque of (timestamp, x, y)
        self._pad_history  = deque(maxlen=VELOCITY_WINDOW)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose', self.pose_callback, 3)

        self.pad_sub = self.create_subscription(
            PoseStamped, '/uav/bottom/pad_aruco_pose', self.pad_callback, 3)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 3)

        # ── Service clients ───────────────────────────────────────────────────
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')
        self.takeoff_client  = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
        self.land_client     = self.create_client(CommandTOL,  '/mavros/cmd/land')

        self.get_logger().info('Waiting for MAVROS services...')
        for client in [
            self.arming_client,
            self.set_mode_client,
            self.takeoff_client,
            self.land_client,
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service...')
        self.get_logger().info('MAVROS services ready.')

        # ── Logging ───────────────────────────────────────────────────────────
        self._log_file   = None
        self._csv_writer = None
        self._init_log()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        if self.initial_pose is None:
            self.initial_pose = msg
            self.get_logger().info('Initial pose latched.')

    def pad_callback(self, msg: PoseStamped):
        self.pad_pose = msg
        t = time.time()
        self._pad_history.append((t, msg.pose.position.x, msg.pose.position.y))

    # ── Logging ───────────────────────────────────────────────────────────────

    def _init_log(self):
        stamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
        node_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir  = os.path.join(node_dir, 'logs', stamp)
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, 'hybrid_landing.csv')
        self._log_file   = open(log_path, 'w', newline='')
        self._csv_writer = csv.writer(self._log_file)
        self._csv_writer.writerow(
            ['operation', 'second', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        self.get_logger().info(f'Logging to {log_path}')

    def _log(self, operation: str):
        if self.current_pose is None or self._csv_writer is None:
            return
        p = self.current_pose.pose.position
        q = self.current_pose.pose.orientation
        roll, pitch, yaw = self._euler_from_quaternion(q.x, q.y, q.z, q.w)
        elapsed = round(time.time() - self.mission_start, 1) if self.mission_start else 0.0
        self._csv_writer.writerow(
            [operation, elapsed, p.x, p.y, p.z, roll, pitch, yaw])

    def _close_log(self):
        if self._log_file:
            self._log_file.close()

    # ── Math helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _euler_from_quaternion(x, y, z, w):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll      = math.atan2(sinr_cosp, cosr_cosp)
        sinp      = 2.0 * (w * y - z * x)
        pitch     = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw       = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _pad_world_xy(self):
        """
        Return pad position in world frame as (x, y).
        If PAD_POSE_RELATIVE, offsets by initial_pose. Otherwise uses directly.
        Returns (None, None) if data unavailable.
        """
        if self.pad_pose is None:
            return None, None

        px = self.pad_pose.pose.position.x
        py = self.pad_pose.pose.position.y

        if PAD_POSE_RELATIVE and self.initial_pose is not None:
            px += self.initial_pose.pose.position.x
            py += self.initial_pose.pose.position.y

        return px, py

    def _estimate_pad_velocity(self):
        """Rolling average velocity from pad pose history. Returns (vx, vy)."""
        if len(self._pad_history) < 2:
            return 0.0, 0.0
        buf = list(self._pad_history)
        dt  = buf[-1][0] - buf[0][0]
        if dt < 1e-6:
            return 0.0, 0.0
        vx = (buf[-1][1] - buf[0][1]) / dt
        vy = (buf[-1][2] - buf[0][2]) / dt
        return vx, vy

    # ── MAVROS primitives ─────────────────────────────────────────────────────

    def ros_sleep(self, duration_sec: float):
        end = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_initial_pose(self, timeout_sec=5.0) -> bool:
        start = time.monotonic()
        while rclpy.ok() and self.initial_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout_sec:
                return False
        return True

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'Mode set to {mode}')
            return True
        self.get_logger().warn(f'Failed to set mode {mode}')
        return False

    def arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Vehicle armed.')
            return True
        self.get_logger().warn('Arming failed.')
        return False

    def takeoff(self, altitude: float) -> bool:
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw       = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0
        req.altitude  = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Takeoff to {altitude}m.')
            return True
        self.get_logger().warn('Takeoff failed.')
        return False

    def land(self) -> bool:
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw       = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0
        req.altitude  = 0.0
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Landing command sent.')
            return True
        self.get_logger().warn('Land failed.')
        return False

    def publish_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish one setpoint to /goal_pose."""
        msg = PoseStamped()
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.pose.position.x    = x
        msg.pose.position.y    = y
        msg.pose.position.z    = z
        msg.pose.orientation.z = yaw
        self.goal_pose_pub.publish(msg)

    # ── Phase logic ───────────────────────────────────────────────────────────

    def hover(self, duration: float, z: float):
        """Hold current XY at z for duration seconds at TICK_RATE_HZ."""
        if self.current_pose is None:
            self.get_logger().warn('No pose for hover.')
            return

        p         = self.current_pose.pose.position
        q         = self.current_pose.pose.orientation
        _, _, yaw = self._euler_from_quaternion(q.x, q.y, q.z, q.w)

        self.get_logger().info(f'Hovering {duration}s at z={z:.2f}m')
        tick = 1.0 / TICK_RATE_HZ
        end  = time.time() + duration

        while rclpy.ok() and time.time() < end:
            self.publish_setpoint(p.x, p.y, z, yaw)
            self._log('HOVER')
            rclpy.spin_once(self, timeout_sec=tick)

    def phase_shadow(self, float_height: float, pursuit_seconds: float):
        """
        Phase 1: Shadow UGV at float_height for pursuit_seconds.

        Each tick:
          - Resolve pad world XY
          - Yaw toward UGV travel direction (falls back to facing pad)
          - Publish setpoint at (pad_wx, pad_wy, float_height)
          - spin_once, repeat
        """
        self.get_logger().info(
            f'Phase 1 SHADOW: z={float_height:.2f}m  duration={pursuit_seconds:.1f}s')

        tick = 1.0 / TICK_RATE_HZ
        end  = time.time() + pursuit_seconds

        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=tick)

            pad_wx, pad_wy = self._pad_world_xy()

            if pad_wx is None:
                self.get_logger().warn('SHADOW: awaiting pad pose, holding.')
                if self.current_pose is not None:
                    p = self.current_pose.pose.position
                    self.publish_setpoint(p.x, p.y, float_height, 0.0)
                continue

            cp = self.current_pose.pose.position

            vx, vy = self._estimate_pad_velocity()
            if math.hypot(vx, vy) > 0.05:
                target_yaw = math.atan2(vy, vx)
            else:
                target_yaw = math.atan2(pad_wy - cp.y, pad_wx - cp.x)

            dx = pad_wx - cp.x
            dy = pad_wy - cp.y
            dist = math.hypot(dx, dy)
            if dist > 1e-3:
                scale = min(1.0, (XY_SPEED / TICK_RATE_HZ) / dist)
                cmd_x = cp.x + dx * scale
                cmd_y = cp.y + dy * scale
            else:
                cmd_x, cmd_y = pad_wx, pad_wy
            self.publish_setpoint(cmd_x, cmd_y, float_height, target_yaw)
            self._log('SHADOW')

            self.get_logger().info(
                f'SHADOW: pad=({pad_wx:.2f},{pad_wy:.2f})  '
                f'v=({vx:.2f},{vy:.2f})  '
                f'remaining={max(0.0, end - time.time()):.1f}s')

        self.get_logger().info('Phase 1 complete.')

    def phase_descent(self):
        """
        Phase 2: Closure-coupled descent from above UGV.

        target_z accumulates each tick rather than reading from current_pose,
        ensuring continuous descent regardless of FCU lag.

        Each tick:
          - Resolve pad world XY
          - Compute lateral distance and rolling velocity
          - dZ = DESCENT_GAIN * target_z * (lat_speed / lat_dist), clamped
          - target_z -= dz * dt  (persistent accumulator)
          - Publish setpoint at (pad_wx, pad_wy, target_z)
          - spin_once, repeat until target_z <= LAND_Z_THRESHOLD
        """
        self.get_logger().info('Phase 2 DESCENT: beginning closure-coupled descent.')

        tick     = 1.0 / TICK_RATE_HZ
        dt       = tick
        target_z = self.current_pose.pose.position.z  # persistent, accumulates descent

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=tick)

            if self.current_pose is None or self.pad_pose is None:
                continue

            cp = self.current_pose.pose.position

            pad_wx, pad_wy = self._pad_world_xy()
            if pad_wx is None:
                continue

            lat_dist  = math.hypot(pad_wx - cp.x, pad_wy - cp.y)
            vx, vy    = self._estimate_pad_velocity()
            lat_speed = math.hypot(vx, vy)

            closure_ratio = (lat_speed / lat_dist) if lat_dist > 1e-3 else 1.0

            raw_dz   = DESCENT_GAIN * target_z * closure_ratio
            dz       = max(MIN_DESCENT_RATE, min(MAX_DESCENT_RATE, raw_dz))
            new_z    = max(0.0, target_z - dz * dt)
            target_z = new_z  # accumulate — next tick descends from here

            target_yaw = math.atan2(pad_wy - cp.y, pad_wx - cp.x)

            dx = pad_wx - cp.x
            dy = pad_wy - cp.y
            dist = math.hypot(dx, dy)
            if dist > 1e-3:
                scale = min(1.0, (XY_SPEED / TICK_RATE_HZ) / dist)
                cmd_x = cp.x + dx * scale
                cmd_y = cp.y + dy * scale
            else:
                cmd_x, cmd_y = pad_wx, pad_wy
            self.publish_setpoint(cmd_x, cmd_y, target_z, target_yaw)
            self._log('DESCENT')

            self.get_logger().info(
                f'DESCENT: lat={lat_dist:.2f}m  z={target_z:.2f}m  '
                f'dz={dz:.3f}m/s  v=({vx:.2f},{vy:.2f})')

            if target_z <= LAND_Z_THRESHOLD:
                self.get_logger().info('Threshold reached — landing.')
                self.land()
                break


def main(args=None):
    rclpy.init(args=args)

    try:
        node = HybridLanding()

        node.get_logger().info('Waiting for initial pose...')
        if not node.wait_for_initial_pose(timeout_sec=5.0):
            node.get_logger().error('No initial pose received. Exiting.')
            return

        node.get_logger().info('Setting GUIDED mode...')
        if not node.set_mode('GUIDED'):
            node.get_logger().error('Failed to set GUIDED. Exiting.')
            return
        node.ros_sleep(1.0)

        node.get_logger().info('Arming...')
        if not node.arm():
            node.get_logger().error('Arming failed. Exiting.')
            return
        node.ros_sleep(1.0)

        node.get_logger().info(f'Taking off to {FLOAT_HEIGHT}m...')
        if not node.takeoff(FLOAT_HEIGHT):
            node.get_logger().error('Takeoff failed. Landing.')
            node.land()
            return
        node.ros_sleep(5.0)

        node.mission_start = time.time()

        # Unit test: stability hover
        node.hover(HOVER_DURATION, FLOAT_HEIGHT)

        # Wait for pad pose before entering sequence
        node.get_logger().info('Waiting for pad pose...')
        timeout = time.time() + 3.0
        while node.pad_pose is None and time.time() < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pad_pose is None:
            node.get_logger().error('No pad pose received. Landing.')
            node.land()
            return

        # Phase 1: shadow
        node.phase_shadow(FLOAT_HEIGHT, PURSUIT_SECONDS)

        node.get_logger().info('Landing')

        # Phase 2: descent
        node.phase_descent()

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node._close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
