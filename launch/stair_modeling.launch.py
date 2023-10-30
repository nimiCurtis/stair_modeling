import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

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

    # Set launch of stair detector
    stair_modeling = Node(
                                package=package_name,
                                executable='stair_modeling',
                                namespace=robot,
                                output='screen',
                                parameters=[config,
                                            {'debug': LaunchConfiguration('debug')}],
                        )

    # Return launch description
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='{time} [{name}] [{severity}] {message}'),
        debug_arg,
        stair_modeling
    ])