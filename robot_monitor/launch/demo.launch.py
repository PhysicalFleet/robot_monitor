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

"""
A demo robot full of typical bugs, plus Robot Monitor.

ros2 launch robot_monitor demo.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

ROLES = ['camera', 'lidar', 'slam', 'detector', 'odometry', 'planner', 'mapper', 'imu',
         'driver', 'driver', 'arm']


def generate_launch_description():
    nodes = [Node(package='robot_monitor', executable='demo_robot', arguments=['--role', r],
                  output='log') for r in ROLES]
    nodes.append(Node(package='robot_monitor', executable='robot_monitor', name='robot_monitor',
                      output='screen'))
    return LaunchDescription(nodes)
