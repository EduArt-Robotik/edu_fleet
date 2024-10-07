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
  use_fleet_control_arg = DeclareLaunchArgument('use_fleet_control', default_value='True')

  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value='eduard')

  # Bring Up Fleet Control Node
  package_path = FindPackageShare('edu_fleet')
  # parameter_file = PathJoinSubstitution([
  #   package_path,
  #   'parameter',
  #   'eduard-fleet-drive-master.yaml'
  # ])

  robot_fleet_control = Node(
    package='edu_fleet',
    executable='robot_localization_node',
    name='robot_localization',
    # parameters=[parameter_file],
    remappings=[
      ('imu', 'imu'),
      ('odometry', 'odometry'),
      ('pose', 'pose')
    ],
    namespace=edu_robot_namespace,
    # prefix=['gdbserver localhost:3000'],
    output='screen'    
  )

  return LaunchDescription([
    use_fleet_control_arg,
    edu_robot_namespace_arg,
    robot_fleet_control
  ])
    