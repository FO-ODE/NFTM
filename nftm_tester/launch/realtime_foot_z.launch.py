from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    foot_marker_topic = LaunchConfiguration('foot_marker_topic')
    foot_labels = LaunchConfiguration('foot_labels')
    window_s = LaunchConfiguration('window_s')
    time_scale = LaunchConfiguration('time_scale')
    update_interval_ms = LaunchConfiguration('update_interval_ms')
    y_min = LaunchConfiguration('y_min')
    y_max = LaunchConfiguration('y_max')
    z_scale = LaunchConfiguration('z_scale')
    z_unit = LaunchConfiguration('z_unit')

    return LaunchDescription([
        DeclareLaunchArgument(
            'foot_marker_topic',
            default_value='/fastlio2/foot_markers',
            description='visualization_msgs/MarkerArray topic containing four FASTLIO2 foot markers.',
        ),
        DeclareLaunchArgument(
            'foot_labels',
            default_value='[fr, fl, rr, rl]',
            description='Labels for the four foot z curves, in marker id order.',
        ),
        DeclareLaunchArgument(
            'window_s',
            default_value='5.0',
            description='Rolling plot window width in seconds.',
        ),
        DeclareLaunchArgument(
            'time_scale',
            default_value='1.0',
            description='Scale applied to elapsed time.',
        ),
        DeclareLaunchArgument(
            'update_interval_ms',
            default_value='30',
            description='Matplotlib refresh interval in milliseconds.',
        ),
        DeclareLaunchArgument(
            'y_min',
            default_value='-30.0',
            description='Fixed minimum value for the y axis.',
        ),
        DeclareLaunchArgument(
            'y_max',
            default_value='-10.0',
            description='Fixed maximum value for the y axis.',
        ),
        DeclareLaunchArgument(
            'z_scale',
            default_value='100.0',
            description='Scale applied to marker pose.position.z before plotting.',
        ),
        DeclareLaunchArgument(
            'z_unit',
            default_value='cm',
            description='Unit label shown on the y axis after applying z_scale.',
        ),
        Node(
            package='nftm_tester',
            executable='realtime_foot_z.py',
            name='realtime_foot_z',
            output='screen',
            parameters=[{
                'foot_marker_topic': foot_marker_topic,
                'foot_labels': foot_labels,
                'window_s': window_s,
                'time_scale': time_scale,
                'update_interval_ms': update_interval_ms,
                'y_min': y_min,
                'y_max': y_max,
                'z_scale': z_scale,
                'z_unit': z_unit,
            }],
        ),
    ])
