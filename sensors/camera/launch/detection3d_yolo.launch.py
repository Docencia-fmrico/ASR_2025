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
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('camera')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    detector_cmd = Node(package='camera',
                        executable='yolo_detection',
                        output='screen',
                        parameters=[param_file],
                        remappings=[
                          ('input_detection', '/yolo/detections'),
                          ('output_detection_2d', '/detections_2d')
                        ])

    convert_2d_3d = Node(package='camera',
                        executable='detection_2d_to_3d_depth',
                        output='screen',
                        parameters=[param_file],
                        remappings=[
                          ('input_depth', '/stereo/depth'),
                          ('input_detection_2d', '/detections_2d'),
                          ('camera_info', '/rgb/camera_info'),
                          ('output_detection_3d', '/detections_3d'),
                        ])

    ld = LaunchDescription()
    ld.add_action(detector_cmd)
    ld.add_action(convert_2d_3d)

    return ld
