from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])