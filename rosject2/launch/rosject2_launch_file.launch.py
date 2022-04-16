from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosject2',
            executable='findwall_server',
            output='screen'),
    ])
