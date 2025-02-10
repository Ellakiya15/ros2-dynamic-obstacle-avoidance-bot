import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('bot_nav'),'config','controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('bot_nav'),'config','bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('bot_nav'), 'config','planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('bot_nav'), 'config', 'recovery.yaml')
    wp_follower = os.path.join(get_package_share_directory('bot_nav'), 'config', 'waypoints.yaml')
    return LaunchDescription([

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            output='screen'),
        
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower_node',
            output='screen',
            parameters=[wp_follower]),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),
 
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server', 
                                        'controller_server',
                                        'behavior_server',
                                        'waypoint_follower_node',
                                        'bt_navigator'
                                        ]}])
    ])