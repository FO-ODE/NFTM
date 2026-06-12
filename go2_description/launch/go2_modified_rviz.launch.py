import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('go2_description')
    urdf_path = os.path.join(package_dir, 'urdf', 'go2_description.urdf')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'go2_fastlio.rviz')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='go2_description',
            executable='lowstate_joint_state_publisher.py',
            name='lowstate_joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'publish_frequency': 100.0
            }],
        ),
        Node(
            package='go2_description',
            executable='fastlio_lidar_to_base_tf.py',
            name='fastlio_lidar_to_base_tf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_rate': 50.0,
            }],
        ),
        Node(
            package='go2_description',
            executable='livox_custom_to_pointcloud2.py',
            name='livox_custom_to_pointcloud2',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])
