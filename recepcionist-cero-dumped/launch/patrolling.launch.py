# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    navigation_dir = get_package_share_directory('ir_robots')

    navigation_cmd = IncludeLaunchDescription(
     PythonLaunchDescriptionSource(os.path.join(navigation_dir,
                                   'launch', 'navigation.launch.py')))
    
    darknet_dir = get_package_share_directory('darknet_ros')

    darknet_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(darknet_dir,
                                    'launch', 'darknet_ros.launch.py')))
    
    darknet_dir = get_package_share_directory('gb_dialog')

    dialog_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(darknet_dir,
                                    'launch', 'gb_dialog_services_soundplay.launch.py')))
    
    


    patrolling_cmd = Node(
        package='recepcionist',
        executable='bt_recepcionist_main',
        parameters=[{
          'use_sim_time': False
        }],
        remappings=[
          ('input_scan', '/scan_raw'),
          ('output_vel', '/cmd_vel')
        ],
        output='screen'
    )

        
    detection2dto3ddepth_cmd = Node(
        package='seekandcapture',
        executable='detection_2d_to_3d_depth',
        parameters=[{
          'use_sim_time': False
        }],
        remappings=[
          ('input_depth', '/camera/depth/image_raw'),
          ('input_detection_2d', '/output_detection_2d'),
          ('output_detection_3d', '/output_detection_3d'),
          ('camera_info', '/camera/depth/camera_info')
        ],
        output='screen'
    )


    perception_cmd = Node(
        package='seekandcapture',
        executable='darknet_detection',
        parameters=[{
          'use_sim_time': False
        }],
        remappings=[
          ('input_bbxs_detection', '/darknet_ros/bounding_boxes'),
          ('output_detection_2d', '/output_detection_2d')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add any actions
    
    ld.add_action(patrolling_cmd)
    ld.add_action(detection2dto3ddepth_cmd)
    ld.add_action(darknet_cmd)
    ld.add_action(perception_cmd)
    ld.add_action(dialog_cmd)
    # ld.add_action(navigation_cmd)
    
    return ld
