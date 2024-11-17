import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    vel_pub = Node(
        package='vel_pub_sub',
        executable='vel_pub',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    vel_sub = Node(
        package='vel_pub_sub',
        executable='vel_sub',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(vel_pub)  
    ld.add_action(vel_sub)

    return ld
    
        