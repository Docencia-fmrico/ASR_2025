# Copyright 2024 Intelligent Robotics Lab
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    pkg_dir = get_package_share_directory('camera')

    params_file = LaunchConfiguration('params_file')
    camera_image_topic = LaunchConfiguration('camera_image_topic')
    camera_depth_topic = LaunchConfiguration('camera_depth_topic')
    camera_points_topic = LaunchConfiguration('camera_points_topic')
    camerainfo_topic = LaunchConfiguration('camerainfo_topic')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_image_topic_cmd = DeclareLaunchArgument(
        'camera_image_topic',
        default_value='/camera/image_raw',
        description='Topic of the rgb image')

    declare_depth_topic_cmd = DeclareLaunchArgument(
        'camera_depth_topic',
        default_value='/camera/depth/image_raw',
        description='Topic of the depth image')

    declare_points_topic_cmd = DeclareLaunchArgument(
        'camera_points_topic',
        default_value='/camera/points',
        description='Topic of the depth image')

    declare_camerainfo_topic_cmd = DeclareLaunchArgument(
        'camerainfo_topic',
        default_value='/camera/camera_info',
        description='Topic of the camera info')

    detector_cmd = Node(package='camera',
                        executable='hsv_filter',
                        output='screen',
                        parameters=[params_file],
                        remappings=[
                          ('input_image', camera_image_topic),
                          ('camera_info', camerainfo_topic),
                          ('output_detection_2d', 'detection_2d'),
                        ])

    convert_2d_3d = Node(package='camera',
                        executable='detection_2d_to_3d_pc2',
                        output='screen',
                        parameters=[params_file],
                        remappings=[
                          # ('input_depth', camera_image_topic),
                          ('input_pointcloud', camera_points_topic),
                          ('input_detection_2d', 'detection_2d'),
                          ('camera_info', camerainfo_topic),
                          ('output_detection_3d', 'detection_3d'),
                        ])

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_depth_topic_cmd)
    ld.add_action(declare_points_topic_cmd)
    ld.add_action(declare_camerainfo_topic_cmd)
    ld.add_action(detector_cmd)
    
    ld.add_action(convert_2d_3d)

    return ld