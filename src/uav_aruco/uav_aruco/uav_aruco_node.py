import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2

MARKER_LENGTH = 0.3048
GOAL_ID = {1,2,3,4}
LANDING_ID = 0

class ArUcoDetector(Node):

    def __init__(self):

        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        self.current_pose = None 
        self.initial_pose = None

        self.declare_parameter('bottom_cam_intr', '')
        self.declare_parameter('front_cam_intr', '')
        self.declare_parameter('uav_loc_topic', '/mavros/vision_pose/pose')

        self.declare_parameter('bottom_img_topic','/bottom_img_topic')
        self.declare_parameter('front_img_topic','/front_img_topic')

        bottom_cam_intr = self.get_parameter('bottom_cam_intr').get_parameter_value().string_value
        front_cam_intr = self.get_parameter('front_cam_intr').get_parameter_value().string_value
        uav_loc_topic = self.get_parameter('uav_loc_topic').get_parameter_value().string_value

        bottom_img_topic = self.get_parameter('bottom_img_topic').get_parameter_value().string_value
        front_img_topic = self.get_parameter('front_img_topic').get_parameter_value().string_value

        self.bottom_matrix_coefficients, self.bottom_distortion_coefficients = self.get_camera_matrices(
            bottom_cam_intr,
            'bottom_cam_intr',
        )
        self.front_matrix_coefficients, self.front_distortion_coefficients = self.get_camera_matrices(
            front_cam_intr,
            'front_cam_intr',
        )
        

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
        q_initial = R.from_quat([
            self.initial_pose.pose.orientation.x,
            self.initial_pose.pose.orientation.y,
            self.initial_pose.pose.orientation.z,
            self.initial_pose.pose.orientation.w,
        ])

        q_current = R.from_quat([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w,
        ])

        # Rotation from current drone frame into initial/origin frame
        r_origin_drone = q_initial.inv() * q_current

        # Rotate marker vector into origin frame and add drone position
        p_origin_marker = p_origin_drone + r_origin_drone.apply(t_vec_drone)

        return p_origin_marker

    def get_camera_matrices(self, path_to_calibration, parameter_name):

        if not path_to_calibration:
            raise ValueError(
                f"Parameter '{parameter_name}' is empty. Set it to a valid OpenCV calibration file."
            )

        fs = cv2.FileStorage(path_to_calibration, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise RuntimeError(
                f"Could not open calibration file '{path_to_calibration}' from parameter '{parameter_name}'."
            )

        camera_matrix_node = fs.getNode('camera_matrix')
        dist_coeffs_node = fs.getNode('distortion_coeffs')
        camera_matrix = camera_matrix_node.mat() if not camera_matrix_node.empty() else None
        dist_coeffs = dist_coeffs_node.mat() if not dist_coeffs_node.empty() else None

        fs.release()

        if camera_matrix is None or camera_matrix.size == 0:
            raise RuntimeError(
                f"Calibration file '{path_to_calibration}' is missing 'camera_matrix'."
            )

        if dist_coeffs is None or dist_coeffs.size == 0:
            raise RuntimeError(
                f"Calibration file '{path_to_calibration}' is missing 'distortion_coeffs'."
            )

        return camera_matrix, dist_coeffs

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
