import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rgb_filter_analyse = Node(
        package='projet',
        executable='color_tuner',
        name='rgb_filter_analyse',
        output='screen'
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(rgb_filter_analyse)

    return ld
