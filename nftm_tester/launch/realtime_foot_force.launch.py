from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lowstate_topic = LaunchConfiguration('lowstate_topic')
    livox_imu_topic = LaunchConfiguration('livox_imu_topic')
    livox_accel_scale = LaunchConfiguration('livox_accel_scale')
    go2_accel_scale = LaunchConfiguration('go2_accel_scale')
    window_s = LaunchConfiguration('window_s')
    time_scale = LaunchConfiguration('time_scale')
    update_interval_ms = LaunchConfiguration('update_interval_ms')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lowstate_topic',
            default_value='/lowstate',
            description='LowState topic containing foot_force and accelerometer data.',
        ),
        DeclareLaunchArgument(
            'livox_imu_topic',
            default_value='/livox/imu',
            description='Livox sensor_msgs/Imu topic to compare with GO2 IMU.',
        ),
        DeclareLaunchArgument(
            'livox_accel_scale',
            default_value='9.80665',
            description='Scale factor applied to Livox IMU acceleration magnitude.',
        ),
        DeclareLaunchArgument(
            'go2_accel_scale',
            default_value='1.04',
            description='Scale factor applied to GO2 IMU acceleration magnitude.',
        ),
        DeclareLaunchArgument(
            'window_s',
            default_value='0.5',
            description='Rolling plot window width in seconds.',
        ),
        DeclareLaunchArgument(
            'time_scale',
            default_value='0.1',
            description='Scale applied to elapsed time. Use 0.2 to make plotted time advance at 0.2x.',
        ),
        DeclareLaunchArgument(
            'update_interval_ms',
            default_value='10',
            description='Matplotlib refresh interval in milliseconds.',
        ),
        Node(
            package='nftm_tester',
            executable='realtime_foot_force.py',
            name='realtime_foot_force',
            output='screen',
            parameters=[{
                'lowstate_topic': lowstate_topic,
                'livox_imu_topic': livox_imu_topic,
                'livox_accel_scale': livox_accel_scale,
                'go2_accel_scale': go2_accel_scale,
                'window_s': window_s,
                'time_scale': time_scale,
                'update_interval_ms': update_interval_ms,
            }],
        ),
    ])
