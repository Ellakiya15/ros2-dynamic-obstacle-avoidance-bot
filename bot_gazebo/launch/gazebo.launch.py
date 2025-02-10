#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    robot_name = DeclareLaunchArgument('robot_name', default_value='bot')
    robot_model = DeclareLaunchArgument('robot_model', default_value='diff_model')
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    world_file_name = 'farm.world'
    pkg_dir = get_package_share_directory('bot_gazebo')
    description_package_name = "bot_description"
    install_dir = get_package_prefix(description_package_name)
 
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
    os.environ["GAZEBO_PLUGIN_PATH"] = os.path.join(install_dir, '/lib')
    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')
 
    gazebo = ExecuteProcess(
                cmd=['gazebo', '--verbose', world, 
                     '-s', 'libgazebo_ros_init.so', 
                     '-s', 'libgazebo_ros_factory.so'],
                output='screen', emulate_tty=True)

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                    'bot',
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )
    return LaunchDescription([
        gazebo,
        spawn_robot
    ])