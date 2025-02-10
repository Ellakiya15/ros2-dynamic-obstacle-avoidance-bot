import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = 'urdf/bot.urdf'
    package_desc = 'bot_description'
    print("URDF LOADING..")
    robot_desc_path = os.path.join(get_package_share_directory(package_desc), urdf_file)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty = True,
        parameters=[{ 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output = "screen"
    )
    rviz_config_dir = os.path.join(get_package_share_directory(package_desc), 'rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])
    
    return LaunchDescription(
        [
            robot_state_publisher_node
        ]
    )