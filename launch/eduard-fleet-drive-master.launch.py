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
        # Robot 0, Eduard Blue, Reference Robot
        ('/cmd_vel'       , 'eduard/blue/cmd_vel'), # use joy stick input from Eduard blue
        ('robot_0/cmd_vel', 'eduard/blue/autonomous/cmd_vel'), # no twist accumulator, connect directly to robot twist input
        ('robot_0/kinematic_description', 'eduard/blue/robot_kinematic_description'),
        ('robot_0/status_report', 'eduard/blue/status_report'),
        # Robot 1, Eduard Green
        ('robot_1/cmd_vel', 'eduard/green/fleet_control/cmd_vel'),
        ('robot_1/kinematic_description', 'eduard/green/robot_kinematic_description'),
        ('robot_1/twist_compensation', 'eduard/green/pose_controller/cmd_vel'),
        ('robot_1/set_lighting_color', 'eduard/green/set_lighting_color'),
        ('robot_1/status_report', 'eduard/green/status_report'),
        # Robot 2, Eduard Red
        ('robot_2/cmd_vel', 'eduard/red/fleet_control/cmd_vel'),
        ('robot_2/kinematic_description', 'eduard/red/robot_kinematic_description'), 
        ('robot_2/twist_compensation', 'eduard/red/pose_controller/cmd_vel'),
        ('robot_2/set_lighting_color', 'eduard/red/set_lighting_color'),
        ('robot_2/status_report', 'eduard/red/status_report')     
      ],
      condition=IfCondition(use_fleet_control),
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )
  
    tf_publisher_qr_code_rear = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
#        '-0.17', '0', '0', '1.570796327', '3.141592654', '-1.570796327',
        '-0.17', '0', '0', '1.570796327', '1.570796327', '0',
        # '--x -0.17', '--y 0', '--z 0', '--roll -1.570796327', '--pitch 0', '--yaw 1.570796327',
        PathJoinSubstitution(['eduard/blue', 'base_link']),
        PathJoinSubstitution(['eduard/blue', 'qr_code', 'rear'])
      ]
    )

    return LaunchDescription([
      use_fleet_control_arg,
      fleet_control_node,
      tf_publisher_qr_code_rear
    ])
    