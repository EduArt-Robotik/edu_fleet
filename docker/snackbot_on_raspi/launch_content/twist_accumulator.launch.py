import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  # Launch File Arguments
  robot_namespace = EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard")
  robot_namespace_arg = DeclareLaunchArgument(
    'robot_namespace', default_value='robot'
  )

  # Twist Accumulation
  twist_accumulator = Node(
    package='edu_fleet',
    executable='twist_accumulator',
    name='twist_accumulator',
    namespace=robot_namespace,
    parameters=[{
      'num_subscriptions': 2,
      'timeout': 0.3
    }],
    remappings=[
      ('twist/input_0', 'navigation/null_cmd_vel'),
      ('twist/input_1', 'path_following/cmd_vel'),
      ('twist/output', 'autonomous/cmd_vel')
    ],
    # prefix=['gdbserver localhost:3000'],
    output='screen'
  )
  
  # Publish a zero Twist message to the navigation/null_cmd_vel topic
  twist_publisher = ExecuteProcess(
    cmd=[
      'ros2', 'topic', 'pub', '-r', '10',
      PathJoinSubstitution(['/', robot_namespace, 'navigation', 'null_cmd_vel']),
      'geometry_msgs/msg/Twist',
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
    ],
    output='log'
  )

  return LaunchDescription([
    robot_namespace_arg,
    twist_accumulator,
    twist_publisher
  ])


