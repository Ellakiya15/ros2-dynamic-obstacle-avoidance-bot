from launch import LaunchDescription
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_param =  os.path.join(get_package_share_directory("bot_nav"),'config','mapper_async.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_sync_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{'use_sim_time': use_sim_time},pkg_param],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(start_sync_slam_toolbox_node)

    return ld