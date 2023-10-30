import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

### TODO: keep writing this lunch file 
## getting : 1698578516.904574225 [zion.zion_container] [ERROR] Failed to load library: 
# Could not load library LoadLibrary error:
# /home/nimrod/ros2_ws/install/zion_components/lib/libcloud_processor_component.so:
# undefined symbol:
# _ZN3pcl14ExtractIndicesINS_11PointXYZRGBEE11applyFilterERNS_10PointCloudIS1_EE,
# at /tmp/binarydeb/ros-foxy-rcutils-1.1.5/src/shared_library.c:84

def generate_launch_description():
    
    package_name = 'stair_modeling'
    components_package_name = 'zion_components'
    # define ns as robot name.
    robot = LaunchConfiguration('robot',default="zion")
    # node_name = LaunchConfiguration('node_name',default="stair_modeling")
    
    # Get parameters from yaml
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'stair_modeling_compose.yaml'
    )

    # Declare the "debug" argument
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Debug mode for stair modeling node'
    )
    
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='zion_container',
            namespace=robot,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     package=components_package_name,
                #     # namespace=robot,
                #     plugin='zion::ZionBroadcaster',
                #     name='zion_broadcaster',
                #     parameters=[config],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package=components_package_name,
                    # namespace=robot,
                    plugin='zion::StairModeling',
                    name='stair_modeling',
                    parameters=[config],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ],
            output='both',
    )
    
    # Return launch description
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='{time} [{name}] [{severity}] {message}'),
        debug_arg,
        container
    ])