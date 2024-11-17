import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    vel_from_laser = Node(
        package='n_garciam_2021_asr',
        executable='vel_from_laser',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    

    ld = LaunchDescription()

    ld.add_action(vel_from_laser)

    return ld
    