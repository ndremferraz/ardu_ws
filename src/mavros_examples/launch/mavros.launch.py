from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

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
            package='topic_tools',
            executable='relay',
            name='zed_pose_to_mavros_vision_pose',
            output='screen',
            arguments=['/zed/zed_node/pose', '/mavros/vision_pose/pose'],
        ),
        Node(
            package='goal_to_cmd_vel',
            executable='controller_node',
            namespace='controller_node',
            output='screen',
        ),
    ])
