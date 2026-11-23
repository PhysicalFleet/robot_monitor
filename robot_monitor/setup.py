# Copyright 2026 PhysicalFleet
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

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['web/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='PhysicalFleet',
    maintainer_email='opensource@physicalfleet.io',
    description='Zero-config health monitor and web dashboard for ROS 2.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'robot_monitor = robot_monitor.main:main',
            'demo_robot = robot_monitor.demo:main',
        ],
        'ros2cli.command': [
            'monitor = robot_monitor.cli.command:MonitorCommand',
        ],
        'ros2cli.extension_point': [
            'robot_monitor.verb = ros2cli.verb:VerbExtension',
        ],
        'robot_monitor.verb': [
            'ui = robot_monitor.cli.command:UiVerb',
            'check = robot_monitor.cli.command:CheckVerb',
            'top = robot_monitor.cli.command:TopVerb',
        ],
    },
)
