import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch File Arguments
    use_fleet_control = LaunchConfiguration('use_fleet_control')
    use_fleet_control_arg = DeclareLaunchArgument(
      'use_fleet_control',
      default_value='True'
    )

    # Bring Up Fleet Control Node
    package_path = FindPackageShare('edu_swarm')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'eduard-fleet-drive-master.yaml'
    ])

    fleet_control_node = Node(
      package='edu_swarm',
      executable='fleet_control_node',
      name='fleet_control_node',
      parameters=[parameter_file],
      remappings=[
        ('robot_0/cmd_vel', 'eduard/red/cmd_vel'),
        ('robot_1/cmd_vel', 'eduard/green/fleet_control/cmd_vel'),
        ('robot_2/cmd_vel', 'eduard/blue/fleet_control/cmd_vel')      
      ],
      condition=IfCondition(use_fleet_control),
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )
    
    return LaunchDescription([
      use_fleet_control_arg,
      fleet_control_node,
    ])
    