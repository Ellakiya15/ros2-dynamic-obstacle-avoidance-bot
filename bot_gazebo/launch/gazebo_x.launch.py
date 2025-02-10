#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')    
    robot_name = DeclareLaunchArgument('robot_name', default_value='agribot')
    robot_model = DeclareLaunchArgument('robot_model', default_value='diff_model')  # jetbot_ros
    
    robot_x = DeclareLaunchArgument('x', default_value='-7.8')
    robot_y = DeclareLaunchArgument('y', default_value='5.0')
    robot_z = DeclareLaunchArgument('z', default_value='0.2')
    
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

    
    spawn_entity = Node(package='bot_gazebo', executable='spawn_action.py',
                        parameters=[
                            {'name': LaunchConfiguration('robot_name')},
                            {'model': LaunchConfiguration('robot_model')},
                            {'x': LaunchConfiguration('x')},
                            {'y': LaunchConfiguration('y')},
                            {'z': LaunchConfiguration('z')},
                        ],
                        output='screen', emulate_tty=True)
 
    return LaunchDescription([
        robot_name,
        robot_model,
        robot_x,
        robot_y,
        robot_z,
        gazebo,
        spawn_entity,
    ])