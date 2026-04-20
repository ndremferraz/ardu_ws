from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pose_source = LaunchConfiguration('pose_source')

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
            default_value='vicon',
            description='Pose source for vins_mav_node: vins or vicon',
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
    ])