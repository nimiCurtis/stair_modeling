# Copyright 2023 Nimrod Curtis
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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    package_name = 'stair_modeling'
    
    # define ns as robot name.
    robot = LaunchConfiguration('robot',default="zion")
    
    # Get parameters from yaml
    config = os.path.join(
        get_package_share_directory('stair_modeling'),
        'config',
        'stair_modeling.yaml'
    )

    # Declare the "debug" argument
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Debug mode for stair modeling node'
    )

    # Declare the "debug" argument
    log_arg = DeclareLaunchArgument(
        'log_level',
        default_value = TextSubstitution(text=str("INFO")),
        description='Logging level'
    )

    # Set launch of tf broadcaster
    broadcaster = Node(
                                package=package_name,
                                executable='zion_broadcaster',
                                namespace=robot,
                                output='screen'
                        )

    # Set launch of stair detector
    stair_modeling = Node(
                                package=package_name,
                                executable='stair_modeling',
                                namespace=robot,
                                output='screen',
                                parameters=[config,
                                            {'debug': LaunchConfiguration('debug')}],
                                arguments=['--ros-args', '--log-level',
                                        LaunchConfiguration('log_level')]
                        )

    # start the detection after 5 secs
    stair_modeling_timer_action = TimerAction(
            period=1.0,
            actions=[stair_modeling]
    )
    
    
    # Return launch description
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT',
                            value='{time} [{name}] [{severity}] {message}'),
        log_arg,
        debug_arg,
        broadcaster,
        stair_modeling_timer_action,
    ])