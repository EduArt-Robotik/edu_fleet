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
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', 'eduard'))

  # Bring Up Line Controller Node


  ## Collision Avoidance
  collision_avoidance_parameter_file = PathJoinSubstitution([
    './',
    'collision_avoidance_lidar.yaml'
  ])
  collision_avoidance = Node(
    package='edu_fleet',
    executable='collision_avoidance_lidar_node',
    name='collision_avoidance_lidar',
    namespace=edu_robot_namespace,
    parameters=[collision_avoidance_parameter_file],
    remappings=[
      # ('in/point_cloud', '/cloud_all_fields_fullframe'),
      ('in/scan', 'scan'),
      ('in/cmd_vel', 'cmd_vel'),
      ('out/cmd_vel', 'safety/cmd_vel')
    ],
    output='screen'
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    collision_avoidance
  ])
