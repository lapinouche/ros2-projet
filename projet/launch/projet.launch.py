import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

# 2.2.7 add teleop node to make the robot mouve 

package_name = "mybot_description"

def generate_launch_description():
    lds_distance_node = Node(
        package='mybot_control',
        executable='lds_distance',
        name='lds_distance'
    )

    stop_emergency_node = Node(
        package='mybot_control',
        executable='stop_emergency',
        name='stop_emergency'
    )

    challange_one_node = Node(
        package='projet',
        executable='projet',
        name='challange_one',
        parameters=[{'use_sim_time': True}]
    )

    pkg_projet_2025 = get_package_share_directory('projet2025')
    
    set_env_vars = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_projet_2025, 'models')
    )

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_projet_2025, 'launch', 'projet.launch.py')
        )
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    # coming from mybot_control
    ld.add_action(lds_distance_node)
    ld.add_action(stop_emergency_node)

    # challange one Node
    ld.add_action(challange_one_node)

    # projet 2025
    ld.add_action(simulation_launch)
    
    return ld
