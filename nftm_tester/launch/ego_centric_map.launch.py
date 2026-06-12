from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    default_config_file = PathJoinSubstitution([
        FindPackageShare('nftm_tester'),
        'config',
        'ego_centric_map.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='YAML parameter file for the base-centered ground clearance grid node.',
        ),
        Node(
            package='nftm_tester',
            executable='ego_centric_map.py',
            name='ego_centric_map',
            output='screen',
            parameters=[config_file],
        ),
    ])
