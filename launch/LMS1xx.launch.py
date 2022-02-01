from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        name="host",
        default_value="192.168.1.14",
    )
    publish_min_range_as_inf_arg = DeclareLaunchArgument(
        name="publish_min_range_as_inf",
        default_value="false",
    )
    lms1xx_node = Node(
        package='lms1xx',
        executable='LMS1xx_node',
        name='lms1xx',
    )
    return LaunchDescription([
        host_arg,
        publish_min_range_as_inf_arg,
        lms1xx_node
    ])
