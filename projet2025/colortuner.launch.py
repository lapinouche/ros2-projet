import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, ExecuteProcess
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
    model_path = os.path.join(os.path.expanduser("~"), "ros2_ws/src/projet2025/models/")
    spawn_random_ball_cmd =ExecuteProcess(
        cmd=["python3", os.path.join(model_path, "Ball/spawn_random_ball.py")],
        output="screen"
    )
    

    spawn_random_goal_cmd =ExecuteProcess(
        cmd=["python3", os.path.join(model_path, "robocup_3Dsim_goal/spawn_random_goal.py")],
        output="screen"
    )
    

    launch_file_dir = os.path.join(get_package_share_directory('projet2025'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_projet2025 = get_package_share_directory('projet2025')

    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.84',
        description='x coordinate of spawned robot'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='-0.05',
        description='y coordinate of spawned robot'
    )

    yaw_angle_arg = DeclareLaunchArgument(
        'yaw_angle', default_value='1.5708',
        description='yaw angle of spawned robot'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='projet.sdf')

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('projet2025'),
                         'models'))

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_projet2025,
            'worlds',
            world_file
        ]),
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        TextSubstitution(text=' -r -v -v1')],
        'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw_angle': LaunchConfiguration('yaw_angle'),
        }.items()
    )

    rgb_filter_analyse = Node(
        package='projet',
        executable='color_tuner',
        name='rgb_filter_analyse',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(spawn_random_ball_cmd)
    ld.add_action(spawn_random_goal_cmd)
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(yaw_angle_arg)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    # Add the commands to the launch description
    ld.add_action(rgb_filter_analyse)

    return ld


# Result pour la simulation : (L - H | L - S | L - V | U - H | U - S | U - V) (avec L = lower et U = upper)
# Green : 60, 30, 80 || 140, 255, 160
# Red : 0, 100, 70 || 10, 255, 255 && 160, 100, 70 || 179, 255, 255
# Blue : 97, 107, 0 || 130, 255, 255

# Valeur maximal pour chaque paramètre : 179, 255, 255, 179, 255, 255
