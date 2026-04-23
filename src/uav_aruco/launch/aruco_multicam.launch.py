from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bottom_cam_intr = LaunchConfiguration('bottom_cam_intr')
    front_cam_intr = LaunchConfiguration('front_cam_intr')
    uav_loc_topic = LaunchConfiguration('uav_loc_topic')
    bottom_img_topic = LaunchConfiguration('bottom_img_topic')
    front_img_topic = LaunchConfiguration('front_img_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bottom_cam_intr',
            default_value='/home/gwcs/ardu_ws/bottom_calib.yaml',
            description='Path to the bottom camera OpenCV calibration file',
        ),
        DeclareLaunchArgument(
            'front_cam_intr',
            default_value='/home/gwcs/ardu_ws/left_calib.yaml',
            description='Path to the front camera OpenCV calibration file',
        ),
        DeclareLaunchArgument(
            'uav_loc_topic',
            default_value='/zed/zed_node/pose',
            description='Pose topic used by the ArUco detector for the UAV pose',
        ),
        DeclareLaunchArgument(
            'bottom_img_topic',
            default_value='/img_publisher',
            description='Image topic for the bottom camera stream',
        ),
        DeclareLaunchArgument(
            'front_img_topic',
            default_value='/zed/zed_node/rgb/color/rect/image',
            description='Image topic for the front camera stream',
        ),
        Node(
            package='uav_aruco',
            executable='uav_aruco_node',
            namespace='uav_aruco',
            output='screen',
            parameters=[
                {
                    'bottom_cam_intr': bottom_cam_intr,
                    'front_cam_intr': front_cam_intr,
                    'uav_loc_topic': uav_loc_topic,
                    'bottom_img_topic': bottom_img_topic,
                    'front_img_topic': front_img_topic,
                }
            ],
        ),
        Node(
            package='multicam',
            executable='multicam_node',
            namespace='multicam_node',
            output='screen',
        ),
    ])
