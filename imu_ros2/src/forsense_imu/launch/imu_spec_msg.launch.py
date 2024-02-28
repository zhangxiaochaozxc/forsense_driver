##launch file
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('imu_ros2'),
        'config',
        'hipnuc_config.yaml',
    ),

    return LaunchDescription([
         Node(
            package='imu_ros2',
            executable='talker',
            name='IMU_publisher',
            parameters=[config],
            output='screen',
            ),
        Node(
            package='imu_ros2',
            executable='listener',
            output='screen'
            ),
        ])

