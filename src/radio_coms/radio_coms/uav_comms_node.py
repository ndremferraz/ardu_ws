#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from pymavlink import mavutil

MARKER_FOUND = 31010   
CURRENT_LOCATION = 31011   
LANDED_SIGNAL = 31012  
SIM_UAV_SEND_PORT = 14560   
SIM_UAV_RECV_PORT = 14561  
SYSID_UAV = 1
SYSID_UGV = 2
SIM_HOST = "127.0.0.1"
 
class UavPeerCommsNode(Node):

    def __init__(self):
        super().__init__("uav_peer_comms_node")

        self.declare_parameter("sim_mode", False)
        self.declare_parameter("mav_device", "")
        self.declare_parameter("location_hz", 1.0)
        self.declare_parameter("mav_send_port", SIM_UAV_SEND_PORT)
        self.declare_parameter("mav_recv_port", SIM_UAV_RECV_PORT)
        self.declare_parameter("mav_host", SIM_HOST)

        self._sim = self.get_parameter("sim_mode").value
        self._loc_hz = self.get_parameter("location_hz").value
        self._send_port = self.get_parameter("mav_send_port").value
        self._recv_port = self.get_parameter("mav_recv_port").value
        self._host = self.get_parameter("mav_host").value

        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()


        self._slam_x: float = 0.0
        self._slam_y: float = 0.0
        self._slam_heading: float = 0.0

        self._mav = self._connect_mavlink()

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/slam_toolbox/pose",
            self._cb_slam_pose,
            10,
            callback_group=self._cbg,
        )

        #vision pipeline publishes detected marker pose here
        self.create_subscription(
            PoseStamped,
            "/target_aruco_pose",
            self._cb_marker_pose,
            10,
            callback_group=self._cbg,
        )

        self._pub_ugv_location = self.create_publisher(
            PoseStamped, "/uav/peer/ugv_location", 10
        )
        self._pub_status = self.create_publisher(
            String, "/uav/peer/status", 10
        )

        self.create_service(
            Trigger, "/uav/peer/landed",
            self._srv_landed, callback_group=self._cbg,
        )

        period = 1.0 / self._loc_hz
        self.create_timer(period, self._broadcast_location, callback_group=self._cbg)

        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True
        )
        self._recv_thread.start()

        self.get_logger().info(
            f"UAV peer comms node started. "
            f"Sim={self._sim}  send=:{self._send_port}  recv=:{self._recv_port}"
        )

    def _connect_mavlink(self) -> mavutil.mavfile:
        device = self.get_parameter("mav_device").value
        if device:
            conn_str = device
        else:
            conn_str = f"udpout:{self._host}:{self._send_port}"
        mav = mavutil.mavlink_connection(
            conn_str,
            source_system=SYSID_UAV,
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
            source_system=SYSID_UAV,
            source_component=1,
        )

    def _recv_loop(self):
        recv_mav = self._connect_mavlink_recv()
        self.get_logger().info("UAV MAVLink recv loop started.")

        while rclpy.ok():
            msg = recv_mav.recv_match(
                type="COMMAND_LONG",
                blocking=True,
                timeout=1.0,
            )
            if msg is None:
                continue
            if msg.target_system not in (SYSID_UAV, 0):
                continue
            self._handle_incoming(msg)

    def _handle_incoming(self, msg):
        cmd = msg.command
        if cmd == CURRENT_LOCATION:
            x = msg.param1
            y = msg.param2
            heading = msg.param3
            self.get_logger().info(
                f"[PEER RX] UGV LOCATION  x={x:.2f}m  y={y:.2f}m  hdg={heading:.1f}°"
            )
            payload = PoseStamped()
            payload.pose.position.x = x
            payload.pose.position.y = y
            self._pub_ugv_location.publish(payload)
        else:
            self.get_logger().warn(f"Unhandled command {cmd} from UGV")

    #recieves marker pose from vision pipeline and sends to UGV
    def _cb_marker_pose(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        q = msg.pose.orientation
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        yaw_deg = math.degrees(yaw_rad) % 360.0

        self.get_logger().info(
            f"[PEER TX] MARKER_FOUND  x={x:.3f}  y={y:.3f}  z={z:.3f}  yaw={yaw_deg:.1f}°"
        )

        self._send_command_long(
            target_system=SYSID_UGV,
            command=MARKER_FOUND,
            param1=1.0,      
            param2=x,
            param3=y,
            param4=z,
            param5=yaw_deg,
        )
        self._publish_status(f"Marker pose sent to UGV ({x:.2f}, {y:.2f}, {z:.2f})")
    #pose of the UAV from SLAM, used for broadcasting to UGV and for debugging
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
    # sends location of the UAV to the UGV at a fixed rate based on whether the marker has been found
    def _broadcast_location(self):
        with self._lock:
            x = self._slam_x    
            y = self._slam_y
            heading = self._slam_heading

        self._send_command_long(
            target_system=SYSID_UGV,
            command=CURRENT_LOCATION,
            param1=x,
            param2=y,
            param3=heading,
        )
        self.get_logger().debug(
            f"[PEER TX] LOCATION  x={x:.2f}m  y={y:.2f}m  hdg={heading:.1f}°"
        )
    #sends landed signal to UGV when UAV has landed
    def _srv_landed(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        ok = self._send_command_long(
            target_system=SYSID_UGV,
            command=LANDED_SIGNAL,  
        )
        response.success = ok
        response.message = "Landed signal sent" if ok else "Send failed"
        self.get_logger().info(f"[PEER TX] LANDED  {response.message}")
        self._publish_status(response.message)
        return response

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

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UavPeerCommsNode()
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
