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

  parameter_file = PathJoinSubstitution([
    './',
    'sick_lidar_custom_reader.yaml'
  ])

  sick_lidar_custom_reader = Node(
    package='edu_perception',
    executable='sick_lidar_custom_reader',
    name='sick_lidar_custom_reader',
    parameters=[parameter_file],
    remappings=[
      ('field_evaluation', 'field_evaluation')
    ],
    namespace=edu_robot_namespace,
    output='screen'
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    sick_lidar_custom_reader
  ])