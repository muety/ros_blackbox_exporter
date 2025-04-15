from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=[FindPackageShare('ros_blackbox_exporter'), '/exporter.yml'],
        description='Path to the configuration YAML file'
    )

    exporter_node = Node(
        package='ros_blackbox_exporter',
        executable='ros_blackbox_exporter',
        name='ros_blackbox_exporter',
        output='screen',
        arguments=['-c', LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        exporter_node
    ])