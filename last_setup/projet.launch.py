import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
     # 2. Definition of your line follower node
    line_follower_node = Node(
        package='projet',
        executable='line_follower',
        name='line_follower_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_line')] # Remap the output away from the real robot
    )

    obstacle_avoidance_node = Node(
        package='projet',
        executable='obstacle_avoidance',
        name='obstacle_avoidance_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_obstacle')]
    )

    corridor_node = Node(
        package='projet',
        executable='corridor_node',
        name='corridor_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_corridor')]
    )

    scoring_node = Node(
        package='projet',
        executable='scoring_node',
        name='scoring_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_scoring')]
    )
    
    node_monitor = Node(
        package='projet',
        executable='node_monitor',
        name='node_monitor',
        output='screen'
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(line_follower_node)
    ld.add_action(obstacle_avoidance_node)
    ld.add_action(corridor_node)
    #ld.add_action(human_motion_control)
    ld.add_action(node_monitor)

    return ld
