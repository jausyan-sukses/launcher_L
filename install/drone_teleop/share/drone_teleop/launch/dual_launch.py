from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_sender',
            executable='control',
            name='control',
            output='screen'
        ),
        Node(
            package='drone_teleop',
            executable='control',
            name='drone_sender',
            output='screen'
        )
    ])
