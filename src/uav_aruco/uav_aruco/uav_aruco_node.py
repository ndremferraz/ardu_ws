import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

MARKER_LENGTH = 0.254
GOAL_ID = {0,1,2,3,4}
LANDING_ID = 5
BOTTOM_CAMERA_MATRIX = np.array([
    [311.2275116435304, 0.0, 302.7510972126],
    [0.0, 311.65181697045796, 225.589028786722],
    [0.0, 0.0, 1.0]
], dtype=np.float64)
BOTTOM_DISTORTION_COEFFICIENTS = np.array([
    [-0.04422166943796803,
     0.01770872063406505,
     0.00012155500254070801,
     0.0005324040497094841,
     0.0]
], dtype=np.float64)
FRONT_CAMERA_MATRIX = np.array([
    [771.815, 0.0, 635.005],
    [0.0, 771.815, 372.985],
    [0.0, 0.0, 1.0]
], dtype=np.float64)
FRONT_DISTORTION_COEFFICIENTS = np.array([
    [-0.0277096,
     -0.0,151386,
     0.00012155500254070801,
     0.0005324040497094841,
     0.0]
], dtype=np.float64)


def quat_xyzw_to_matrix(quat):
    """Convert an xyzw quaternion into a 3x3 rotation matrix."""

    quat = np.asarray(quat, dtype=float)
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        raise ValueError('Received zero-norm quaternion.')

    x, y, z, w = quat / norm

    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])

class ArUcoDetector(Node):

    def __init__(self):

        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        self.current_pose = None 
        self.initial_pose = None

        self.declare_parameter('uav_loc_topic', '/mavros/vision_pose/pose')

        self.declare_parameter('bottom_img_topic','/bottom_img_topic')
        self.declare_parameter('front_img_topic','/front_img_topic')

        uav_loc_topic = self.get_parameter('uav_loc_topic').get_parameter_value().string_value

        bottom_img_topic = self.get_parameter('bottom_img_topic').get_parameter_value().string_value
        front_img_topic = self.get_parameter('front_img_topic').get_parameter_value().string_value

        self.bottom_matrix_coefficients = BOTTOM_CAMERA_MATRIX
        self.bottom_distortion_coefficients = BOTTOM_DISTORTION_COEFFICIENTS
        self.front_matrix_coefficients = FRONT_CAMERA_MATRIX
        self.front_distortion_coefficients = FRONT_DISTORTION_COEFFICIENTS
        

        self.bottom_target_pub = self.create_publisher(PoseStamped, "uav/bottom/target_aruco_pose", 3)
        self.front_target_pub = self.create_publisher(PoseStamped, "uav/front/target_aruco_pose", 3)
        self.bottom_pad_pub = self.create_publisher(PoseStamped, "uav/bottom/pad_aruco_pose", 3)
        self.front_pad_pub = self.create_publisher(PoseStamped, "uav/front/pad_aruco_pose", 3)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            uav_loc_topic, 
            self.pose_callback,
            10
        )

        self.bottom_img_sub = self.create_subscription(Image, bottom_img_topic, self.bottom_img_callback, 3)
        self.front_img_sub = self.create_subscription(Image, front_img_topic, self.front_img_callback, 3)

        self.get_logger().info(f"ArUco Detector Node Started. Listening on image_topic")

    def pose_callback(self, msg: PoseStamped):

        self.current_pose = msg

        if self.initial_pose is None:
            self.initial_pose = msg

    def front_img_callback(self, msg: Image):

        if self.current_pose is None or self.initial_pose is  None:
            return 

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, dictionary, parameters=parameters)


            if ids is not None:
                for i in range(len(ids)):

                    marker_id = int(ids[i][0])
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i],
                        MARKER_LENGTH,
                        self.front_matrix_coefficients,
                        self.front_distortion_coefficients,
                    )
                    
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header

                    tx, ty, tz = tvec[0][0]

                    if marker_id in GOAL_ID:

                        t_vec_enu = np.array([tz,-tx,-ty])

                        t_vec_project = self.marker_to_origin_frame(t_vec_enu)

                        pose_msg.pose.position.x = t_vec_project[0] 
                        pose_msg.pose.position.y = t_vec_project[1] 
                        pose_msg.pose.position.z = t_vec_project[2] 

                        self.front_target_pub.publish(pose_msg)
                        self.get_logger().info(
                            f"Detected ID {marker_id} at Pose [X:{pose_msg.pose.position.x:.2f}m, Y:{pose_msg.pose.position.y:.2f}m, Z:{pose_msg.pose.position.z:.2f}m]"
                        )
                    elif marker_id == LANDING_ID:

                        # Rotating from front camera frame to Mavros ENU Frame
                        pose_msg.pose.position.x = tz
                        pose_msg.pose.position.y = -tx
                        pose_msg.pose.position.z = -ty

                        self.front_pad_pub.publish(pose_msg)
                        self.get_logger().info(
                            f"Detected landing pad ID {marker_id} at Pose [X:{pose_msg.pose.position.x:.2f}m, Y:{pose_msg.pose.position.y:.2f}m, Z:{pose_msg.pose.position.z:.2f}m]"
                        )
                    else:
                        self.get_logger().info(f"Detected unknown marker: {marker_id}")
            
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")



    def bottom_img_callback(self, msg: Image):

        if self.current_pose is None or self.initial_pose is  None:
            return 

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, dictionary, parameters=parameters)


            if ids is not None:
                for i in range(len(ids)):

                    marker_id = int(ids[i][0])
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i],
                        MARKER_LENGTH,
                        self.bottom_matrix_coefficients,
                        self.bottom_distortion_coefficients,
                    )

                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header

                    tx, ty, tz = tvec[0][0]

                    if marker_id in GOAL_ID:

                        t_vec_enu = np.array([-ty,-tx,-tz])
                        
                        t_vec_project = self.marker_to_origin_frame(t_vec_enu)

                        pose_msg.pose.position.x = t_vec_project[0]
                        pose_msg.pose.position.y = t_vec_project[1] 
                        pose_msg.pose.position.z = t_vec_project[2] 

                        self.bottom_target_pub.publish(pose_msg)
                        self.get_logger().info(
                            f"Detected ID {marker_id} at Pose [X:{pose_msg.pose.position.x:.2f}m, Y:{pose_msg.pose.position.y:.2f}m, Z:{pose_msg.pose.position.z:.2f}m]"
                        )
                    elif marker_id == LANDING_ID:

                        # Rotating from bottom camera frame to Mavros ENU Frame
                        pose_msg.pose.position.x = -ty
                        pose_msg.pose.position.y = -tx
                        pose_msg.pose.position.z = -tz

                        self.bottom_pad_pub.publish(pose_msg)
                        self.get_logger().info(
                            f"Detected landing pad ID {marker_id} at Pose [X:{pose_msg.pose.position.x:.2f}m, Y:{pose_msg.pose.position.y:.2f}m, Z:{pose_msg.pose.position.z:.2f}m]"
                        )
                    else:
                        self.get_logger().info(f"Detected unknown marker: {marker_id}")
            
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def marker_to_origin_frame(self, t_vec_drone):
        """
        Convert marker position from drone/body-aligned frame to the initial/origin frame.
        t_vec_drone: marker translation expressed in drone frame.
        """

        # Current drone position relative to initial origin
        p_origin_drone = np.array([
            self.current_pose.pose.position.x - self.initial_pose.pose.position.x,
            self.current_pose.pose.position.y - self.initial_pose.pose.position.y,
            self.current_pose.pose.position.z - self.initial_pose.pose.position.z,
        ])

        # Initial and current drone orientations
        q_initial = np.array([
            self.initial_pose.pose.orientation.x,
            self.initial_pose.pose.orientation.y,
            self.initial_pose.pose.orientation.z,
            self.initial_pose.pose.orientation.w,
        ])

        q_current = np.array([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w,
        ])

        # Rotation from current drone frame into initial/origin frame.
        r_initial_world = quat_xyzw_to_matrix(q_initial).T
        r_world_drone = quat_xyzw_to_matrix(q_current)
        r_origin_drone = r_initial_world @ r_world_drone

        # Rotate marker vector into origin frame and add drone position.
        p_origin_marker = p_origin_drone + (r_origin_drone @ t_vec_drone)

        return p_origin_marker

    def destroy_node(self):
        self.get_logger().info("Shutting down ArUco Detector")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
