from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pose_source = LaunchConfiguration('pose_source')
    bottom_cam_intr = LaunchConfiguration('bottom_cam_intr')
    front_cam_intr = LaunchConfiguration('front_cam_intr')
    uav_loc_topic = LaunchConfiguration('uav_loc_topic')
    bottom_img_topic = LaunchConfiguration('bottom_img_topic')
    front_img_topic = LaunchConfiguration('front_img_topic')
    uav_comms_device = LaunchConfiguration('uav_comms_device')
    uav_comms_sim_mode = LaunchConfiguration('uav_comms_sim_mode')

    config = os.path.join(
        get_package_share_directory('mavros_examples'),
        'config',
        'new_apm_config.yaml'
    )

    plugins = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'apm_pluginlists.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_source',
            default_value='vins',
            description='Pose source for vins_mav_node: vins or vicon',
        ),
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
            default_value='/mavros/vision_pose/pose',
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
        DeclareLaunchArgument(
            'uav_comms_device',
            default_value='/dev/ttyUSB0',
            description='Serial device used by radio_coms uav_comms_node',
        ),
        DeclareLaunchArgument(
            'uav_comms_sim_mode',
            default_value='False',
            description='Whether radio_coms uav_comms_node should run in simulation mode',
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            output='screen',
            parameters=[
                config,
                plugins,
                {
                    'fcu_url': 'serial:///dev/ttyACM0:115200',
                    'gcs_url': '',
                    'tgt_system': 1,
                    'tgt_component': 1,
                    'fcu_protocol': 'v2.0',
                }
            ]
        ), 
        Node(
            package='vins_mav_pose',
            executable='vins_mav_node',
            namespace='vins_mav_pose',
            output='screen',
            parameters=[
                {
                    'pose_source': pose_source,
                }
            ]
        ),
        Node(
            package='goal_to_cmd_vel',
            executable='controller_node',
            namespace='controller_node',
            output='screen',
        ),
        Node(
            package='pose_to_odom',
            executable='pose_to_odom_node',
            namespace='pose_to_odom_node',
            output='screen',
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
            ]
        ),
        Node(
            package='radio_coms',
            executable='uav_comms_node',
            namespace='radio_coms',
            output='screen',
            parameters=[
                {
                    'mav_device': uav_comms_device,
                    'sim_mode': uav_comms_sim_mode,
                }
            ]
        ),
        Node(
            package='multicam',
            executable='multicam_node',
            namespace='multicam_node',
            output='screen',
        ),
    ])
