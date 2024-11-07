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
  ## Line Controller
  line_controller_parameter_file = PathJoinSubstitution([
    './',
    'sick_line_controller.yaml'
  ])

  line_controller = Node(
    package='edu_fleet',
    executable='sick_line_controller_node',
    name='sick_line_controller',
    parameters=[line_controller_parameter_file],
    remappings=[
      ('out/velocity', 'line_controller/cmd_vel'),
      ('out/on_track', 'line_controller/on_track'),
      ('in/line_detection', '/localizationcontroller/out/line_measurement_message_0404'),
    ],
    namespace=edu_robot_namespace,
    # prefix=['gdbserver localhost:3000'],
    output='screen'    
  )

  ## Line Navigation
  line_navigation_parameter_file = PathJoinSubstitution([
    './',
    'sick_line_navigation.yaml'
  ])

  line_navigation = Node(
    package='edu_fleet',
    executable='sick_line_navigation_node',
    name='sick_line_navigation',
    parameters=[line_navigation_parameter_file],
    remappings=[
      ('out/cmd_vel', 'line_navigation/cmd_vel'),
      ('out/set_lighting_color', 'set_lighting_color'),      
      ('in/on_track', 'line_controller/on_track'),
      ('in/code', '/localizationcontroller/out/code_measurement_message_0304')
    ],
    namespace=edu_robot_namespace,
    output='screen'
  )

  ## Twist Accumulation
  twist_accumulator = Node(
    package='edu_fleet',
    executable='twist_accumulator',
    name='twist_accumulator',
    namespace=edu_robot_namespace,
    # parameter=[parameter_file],
    remappings=[
      ('twist/input_0', 'line_controller/cmd_vel'),
      ('twist/input_1', 'line_navigation/cmd_vel'),
      ('twist/output', 'autonomous/cmd_vel')
    ],
    # prefix=['gdbserver localhost:3000'],
    output='screen'
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    line_controller,
    line_navigation,
    twist_accumulator
  ])
    