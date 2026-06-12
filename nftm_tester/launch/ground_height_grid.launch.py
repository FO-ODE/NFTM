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
        'ground_height_grid.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='YAML parameter file for the ground height grid node.',
        ),
        Node(
            package='nftm_tester',
            executable='ground_height_grid.py',
            name='ground_height_grid',
            output='screen',
            parameters=[config_file],
        ),
    ])
