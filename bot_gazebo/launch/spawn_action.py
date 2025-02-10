#! /usr/bin/python3
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory 
from gazebo_msgs.srv import SpawnEntity


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("entity_spawner", namespace="agribot")

    node.declare_parameter('name', 'agribot')
    node.declare_parameter('model', 'agribot_ros')
    node.declare_parameter('x', 0.0)
    node.declare_parameter('y', 0.0)
    node.declare_parameter('z', 0.0)
    
    robot_name = node.get_parameter('name').value
    robot_model = node.get_parameter('model').value
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    # Get the spawn_entity service
    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the robot
    urdf_file_path = os.path.join(get_package_share_directory("bot_description"), "urdf","bot.urdf")

    # Show file path
    print(f"robot_urdf={urdf_file_path}")
    print(f"robot_name={robot_name}")
    
    # Set data for request
    request = SpawnEntity.Request()
    request.name = robot_name
    request.xml = open(urdf_file_path, 'r').read()
    request.robot_namespace = robot_name
    request.initial_pose.position.x = float(node.get_parameter('x').value)
    request.initial_pose.position.y = float(node.get_parameter('y').value)
    request.initial_pose.position.z = float(node.get_parameter('z').value)

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()