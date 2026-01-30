from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
      IncludeLaunchDescription(
        PathJoinSubstitution([
          FindPackageShare('foxglove_bridge'),
            'launch',
            'foxglove_bridge_launch.xml'
        ]),
      ),
      Node(
          package='barracuda_thrusters',
          namespace='barracuda',
          executable='barracuda_thrusters'
      ),
    ])
