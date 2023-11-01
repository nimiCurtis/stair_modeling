# Copyright 2022 Nimrod
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
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'stair_modeling'

    # Rviz2 params file
    config_rviz2 = os.path.join(
        get_package_share_directory(package_name),
            'rviz2',
            'stair_modeling.rviz')
    
    launch_rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=[["-d"], [config_rviz2] ]
        )

    # return launch file
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='{time} [{name}] [{severity}] {message}'),
        launch_rviz
    ])