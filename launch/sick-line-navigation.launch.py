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
    FindPackageShare('edu_fleet'),
    'parameter',
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
      ('in/drive_action', 'drive_action')
    ],
    namespace=edu_robot_namespace,
    # prefix=['gdbserver localhost:3000'],
    output='screen'    
  )

  ## Line Navigation
  line_navigation_parameter_file = PathJoinSubstitution([
    FindPackageShare('edu_fleet'),
    'parameter',
    'sick_line_navigation.yaml'
  ])

  line_navigation = Node(
    package='edu_fleet',
    executable='sick_line_navigation_node',
    name='sick_line_navigation',
    parameters=[
      line_navigation_parameter_file,
      {'line_controller_node_name': 'sick_line_controller'}
    ],
    remappings=[
      ('out/cmd_vel', 'line_navigation/cmd_vel'),
      ('out/set_lighting_color', 'set_lighting_color'),
      ('out/drive_action', 'drive_action'),
      ('in/on_track', 'line_controller/on_track'),
      ('in/code', '/localizationcontroller/out/code_measurement_message_0304'),
      ('in/field_evaluation', 'field_evaluation')
    ],
    namespace=edu_robot_namespace,
    output='screen'
  )

  ## Odometry Repeater for improving sick line navigation performance
  odometry_repeater = Node(
    package='edu_fleet',
    executable='sick_odometry_repeater_node',
    name='sick_odometry_repeater',
    namespace=edu_robot_namespace,
    remappings=[
      ('in/odom', 'odometry'),
      ('out/odom', '/localizationcontroller/in/odometry_message_0104')
    ],
    output='screen'
  )

  ## Rotate Robot Action Server
  # rotate_robot_parameter_file = PathJoinSubstitution([
  #   FindPackageShare('edu_fleet'),
  #   'parameter',
  #   'rotate_robot.yaml'
  # ])

  rotate_robot = Node(
    package='edu_fleet',
    executable='robot_rotate_node',
    name='robot_rotate',
    namespace=edu_robot_namespace,
    # parameters=[rotate_robot_parameter_file],
    remappings=[
      ('in/odometry', 'odometry'),
      ('out/cmd_vel', 'rotate_robot/cmd_vel'),
    ],
    output='screen'
  )  

  ## Twist Accumulation
  twist_accumulator = Node(
    package='edu_fleet',
    executable='twist_accumulator',
    name='twist_accumulator',
    namespace=edu_robot_namespace,
    parameters=[
      {'num_subscription': 3}
    ],
    remappings=[
      ('twist/input_0', 'line_controller/cmd_vel'),
      ('twist/input_1', 'line_navigation/cmd_vel'),
      ('twist/input_2', 'rotate_robot/cmd_vel'),
      ('twist/output', 'combined/cmd_vel')
    ],
    # prefix=['gdbserver localhost:3000'],
    output='screen'
  )

  ## Collision Avoidance
  collision_avoidance_parameter_file = PathJoinSubstitution([
    FindPackageShare('edu_fleet'),
    'parameter',
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
      ('in/scan', '/scan_fullframe'),
      ('in/cmd_vel', 'combined/cmd_vel'),
      ('out/cmd_vel', 'autonomous/cmd_vel')
    ],
    output='screen'
  )

  ## Collision Avoidance Lidar Field
  collision_avoidance_lidar_field = Node(
    package='edu_fleet',
    executable='collision_avoidance_lidar_field_node',
    name='collision_avoidance_lidar_field',
    namespace=edu_robot_namespace,
    # parameters=[],
    remappings=[
      ('in/twist', 'combined/cmd_vel'),
      ('in/field', 'field_evaluation'),
      ('out/twist', 'autonomous/cmd_vel')
    ],
    output='screen'
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    line_controller,
    line_navigation,
    odometry_repeater,
    rotate_robot,
    twist_accumulator,
    # collision_avoidance,
    collision_avoidance_lidar_field
  ])
