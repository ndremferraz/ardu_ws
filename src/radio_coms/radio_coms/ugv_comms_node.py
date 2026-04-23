#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool

from pymavlink import mavutil

MARKER_FOUND = 31010   
CURRENT_LOCATION = 31011   
LANDED_SIGNAL = 31012  
SYSID_UAV = 1
SYSID_UGV = 2
SIM_HOST = "127.0.0.1"
SIM_UGV_RECV_PORT = 14560  
SIM_UGV_SEND_PORT = 14561   



class UgvPeerCommsNode(Node):

    def __init__(self):
        super().__init__("ugv_peer_comms_node")

        self.declare_parameter("sim_mode", False)
        self.declare_parameter("mav_device", "")
        self.declare_parameter("location_hz", 1.0)
        self.declare_parameter("mav_send_port", SIM_UGV_SEND_PORT)
        self.declare_parameter("mav_recv_port", SIM_UGV_RECV_PORT)
        self.declare_parameter("mav_host", SIM_HOST)

        self._sim = self.get_parameter("sim_mode").value
        self._loc_hz = self.get_parameter("location_hz").value
        self._send_port = self.get_parameter("mav_send_port").value
        self._recv_port = self.get_parameter("mav_recv_port").value
        self._host = self.get_parameter("mav_host").value

        self._cbg  = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # marker location from UAV
        self._slam_x: float = 0.0
        self._slam_y: float = 0.0
        self._slam_heading: float = 0.0

        # location broadcast only starts after MARKER_FOUND received
        self._marker_found: bool = False
        self._mav = self._connect_mavlink()

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/slam_toolbox/pose",
            self._cb_slam_pose,
            10,
            callback_group=self._cbg,
        )

        self._pub_marker_found = self.create_publisher(PoseStamped, "/ugv/peer/marker_found", 10)
        self._pub_uav_landed = self.create_publisher( Bool, "/ugv/peer/uav_landed", 10)
        self._pub_uav_location = self.create_publisher( String, "/ugv/peer/uav_location", 10)
        self._pub_status = self.create_publisher( String, "/ugv/peer/status", 10)

        period = 1.0 / self._loc_hz
        self.create_timer(period, self._broadcast_location, callback_group=self._cbg)

        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True
        )
        self._recv_thread.start()

        self.get_logger().info(
            f"UGV peer comms node started. "
            f"Sim={self._sim}  send=:{self._send_port}  recv=:{self._recv_port}  "
            f"Location broadcast: WAITING for marker found"
        )

    def _connect_mavlink(self) -> mavutil.mavfile:
        device = self.get_parameter("mav_device").value
        if device:
            conn_str = device
        else:
            conn_str = f"udpout:{self._host}:{self._send_port}"
        mav = mavutil.mavlink_connection(
            conn_str,
            source_system=SYSID_UGV,
            source_component=1,
            baud=57600,
        )
        self.get_logger().info(f"MAVLink send connection: {conn_str}")
        return mav

    def _connect_mavlink_recv(self) -> mavutil.mavfile:
        device = self.get_parameter("mav_device").value
        if device:
            return self._mav
        conn_str = f"udpin:{self._host}:{self._recv_port}"
        return mavutil.mavlink_connection(
            conn_str,
            source_system=SYSID_UGV,
            source_component=1,
        )

    def _recv_loop(self):
        recv_mav = self._connect_mavlink_recv()
        self.get_logger().info("UGV MAVLink recv loop started.")

        while rclpy.ok():
            msg = recv_mav.recv_match(
                type="COMMAND_LONG",
                blocking=True,
                timeout=1.0,
            )
            if msg is None:
                continue
            if msg.target_system not in (SYSID_UGV, 0):
                continue
            self._handle_incoming(msg)

    def _handle_incoming(self, msg):
        cmd = msg.command
        if cmd == MARKER_FOUND:
            self._on_marker_found(msg)
        elif cmd == CURRENT_LOCATION:
            self._on_uav_location(msg)
        elif cmd == LANDED_SIGNAL:
            self._on_uav_landed(msg)
        else:
            self.get_logger().warn(f"Unhandled command {cmd}")
    #when marker found command is received, it extracts the pose information and republishes it as a PoseStamped message
    # also updates internal state to start broadcasting location
    def _on_marker_found(self, msg):
        marker_id = int(msg.param1)
        x = msg.param2
        y = msg.param3
        z = msg.param4
        yaw_deg = msg.param5

        self.get_logger().info(
            f" MARKER_FOUND  id={marker_id}"
            f"x={x:.3f}  y={y:.3f}  z={z:.3f}  yaw={yaw_deg:.1f}°"
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        yaw_rad = math.radians(yaw_deg)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose_msg.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self._pub_marker_found.publish(pose_msg)
        self._publish_status(
            f"Marker {marker_id} found at ({x:.2f}, {y:.2f}, {z:.2f})"
        )

        with self._lock:
            if not self._marker_found:
                self._marker_found = True
                self.get_logger().info(
                    "[PEER] Marker found — starting continuous location broadcast to UAV"
                )
    # when landed signal is recieved, publishes uav location
    def _on_uav_location(self, msg):
        x = msg.param1
        y = msg.param2
        heading = msg.param3
        self.get_logger().info(
            f"[PEER RX] UAV LOCATION  x={x:.2f}m  y={y:.2f}m  hdg={heading:.1f}°"
        )
        payload = String()
        payload.data = (
            f'{{"source":"uav","x":{x:.3f},"y":{y:.3f},"heading":{heading:.1f}}}'
        )
        self._pub_uav_location.publish(payload)

    def _on_uav_landed(self, msg):
        self.get_logger().info("[PEER RX] UAV LANDED")
        flag = Bool()
        flag.data = True
        self._pub_uav_landed.publish(flag)
        self._publish_status("UAV landed")

    def _send_command_long(
        self,
        target_system: int,
        command: int,
        param1=0.0, param2=0.0, param3=0.0,
        param4=0.0, param5=0.0, param6=0.0, param7=0.0,
    ) -> bool:
        try:
            self._mav.mav.command_long_send(
                target_system,
                0,
                command,
                0,
                param1, param2, param3,
                param4, param5, param6, param7,
            )
            return True
        except Exception as e:
            self.get_logger().error(f"MAVLink send error: {e}")
            return False


    def _cb_slam_pose(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self._slam_x = msg.pose.pose.position.x
            self._slam_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self._slam_heading = math.degrees(yaw) % 360.0


    def _broadcast_location(self):
        with self._lock:
            if not self._marker_found:
                return
            x = self._slam_x
            y = self._slam_y
            heading = self._slam_heading

        self._send_command_long(
            target_system=SYSID_UAV,
            command=CURRENT_LOCATION,
            param1=x,
            param2=y,
            param3=heading,
        )
        self.get_logger().debug(
            f"[PEER TX] LOCATION  x={x:.2f}m  y={y:.2f}m  hdg={heading:.1f}°"
        )

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UgvPeerCommsNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
