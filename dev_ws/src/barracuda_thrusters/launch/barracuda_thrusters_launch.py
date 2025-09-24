from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    n_thrusters = LaunchConfiguration('n_thrusters')
    return LaunchDescription([
        DeclareLaunchArgument(
            'n_thrusters',
            default_value='8'
        ),
        Node(
            package='barracuda_thrusters',
            namespace='barracuda',
            executable='barracuda_thrusters',
            parameters=[{
                'n_thrusters': n_thrusters
            }],
        ),
        # Node(
        #     package='barracuda_thrusters',
        #     namespace='barracuda',
        #     executable='test_publisher',
        # )
    ])