from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    test_thrusters = LaunchConfiguration('test_thrusters')
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'test_thrusters',
        #     default_value='all'
        # ),
        Node(
            package='barracuda_thrusters',
            namespace='barracuda',
            executable='barracuda_thrusters',
            # parameters=[{
            #     'test_thrusters': test_thrusters
            # }],
        )      
    ])