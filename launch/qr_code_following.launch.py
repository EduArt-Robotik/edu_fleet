import os

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # Launch File Arguments
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard'))

  use_sim_time = LaunchConfiguration('use_sim_time')
  use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

  # Marker
  marker_pose_estimator_parameter_file = PathJoinSubstitution([
    FindPackageShare('edu_fleet'),
    'parameter',
    'qr_code_following',
    'marker_pose_estimation.yaml'
  ])
  camera_parameter_file = PathJoinSubstitution([
    FindPackageShare('edu_fleet'),
    'parameter',
    'qr_code_following',
    'camera.yaml'
  ])

  camera_node = Node(
    package='usb_cam',
    executable='usb_cam_node_exe',
    name='camera_node',
    remappings=[
      ('image_raw', 'camera/image'),
      ('camera_info', 'camera/camera_info')
    ],
    parameters=[
      camera_parameter_file,
      { 'use_sim_time': use_sim_time }          
    ],
    namespace=edu_robot_namespace,
    output='screen'
  )
  qr_code_deteciton = Node(
    package='zbar_ros',
    executable='barcode_reader',
    name='qr_code_dection',
    remappings=[
      ('image', 'camera/image'),
      ('symbol', 'marker/symbol')
    ],
    namespace=edu_robot_namespace,
    output='screen'
  )
  marker_pose_estimation = Node(
    package='edu_perception',
    executable='marker_pose_estimation',
    name='marker_pose_estimation',
    remappings=[
      ('camera_info', 'camera/camera_info'),
      ('estimated_pose', 'marker/pose'),
      ('qr_code_detection', 'marker/symbol')
    ],
    parameters=[
      marker_pose_estimator_parameter_file,
      { 'use_sim_time': use_sim_time }
    ],
    namespace=edu_robot_namespace,
    output='screen'
  )

  # Kalman Filter for Pose Filtering
  robot_localization_parameter = PathJoinSubstitution([
    FindPackageShare('edu_fleet'),
    'parameter',
    'qr_code_following',
    'robot_localization.yaml'
  ])

  robot_localization = Node(
    package='edu_fleet',
    executable='robot_localization_node',
    name='robot_localization',
    parameters=[
      robot_localization_parameter,
      {'use_sim_time': use_sim_time},
      {'robot_name': edu_robot_namespace}
    ],
    remappings=[
      ('imu', 'imu'),
      ('odometry', 'odometry'),
      ('pose', 'marker/pose')
    ],
    namespace=edu_robot_namespace,
    # prefix=['gdbserver localhost:3000'],
    output='screen'
  )  

  return LaunchDescription([
    edu_robot_namespace_arg,
    use_sim_time_arg,    
    camera_node,
    marker_pose_estimation,
    qr_code_deteciton,
    robot_localization
  ])