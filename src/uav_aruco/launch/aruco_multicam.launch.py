from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    uav_loc_topic = LaunchConfiguration('uav_loc_topic')
    bottom_img_topic = LaunchConfiguration('bottom_img_topic')
    front_img_topic = LaunchConfiguration('front_img_topic')

    return LaunchDescription([
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
        Node(
            package='uav_aruco',
            executable='uav_aruco_node',
            namespace='uav_aruco',
            output='screen',
            parameters=[
                {
                    'uav_loc_topic': uav_loc_topic,
                    'bottom_img_topic': bottom_img_topic,
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
