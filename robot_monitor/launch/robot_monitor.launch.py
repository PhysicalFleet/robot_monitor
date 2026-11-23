# Copyright 2026 The Robot Monitor Authors
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

"""Launch Robot Monitor next to your robot: ros2 launch robot_monitor robot_monitor.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_monitor')
    cfg = os.path.join(share, 'config', 'robot_monitor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9191'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        Node(
            package='robot_monitor', executable='robot_monitor', name='robot_monitor',
            output='screen',
            parameters=[cfg, {'port': LaunchConfiguration('port'),
                              'host': LaunchConfiguration('host')}],
            respawn=True, respawn_delay=2.0,
        ),
    ])
