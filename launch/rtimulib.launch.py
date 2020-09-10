import os

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    this_prefix = get_package_share_directory('ros2_rtimulib')

    imu_config_path = os.path.join(this_prefix, 'config', 'imu.yaml')

    return LaunchDescription([
        Node(
            package='ros2_rtimulib',
            node_namespace='',
            node_name='imu_node',
            node_executable='imu_node',
            output='screen',
            parameters=[imu_config_path],
            remappings=None,
            arguments=[],
            name="",
            cwd=None,
            env=None
        )
    ])

