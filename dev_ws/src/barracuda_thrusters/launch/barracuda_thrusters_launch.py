from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='barracuda_thrusters',
            namespace='barracuda',
            executable='barracuda_thrusters',
            parameters=[{
                'n_thrusters': 8
            }],
            remappings=[
                ('/scan', '/scan_1')
            ]
        ),
        Node(
            package='barracuda_thrusters',
            executable='test_publisher',
        )
    ])