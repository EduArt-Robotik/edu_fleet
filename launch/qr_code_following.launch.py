import os

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
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
      { 'use_sim_time': use_sim_time },
      { 'frame_id': PathJoinSubstitution([edu_robot_namespace, 'marker_camera'])}    
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
  tf_camera_transform = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
      '0.2', '0.0', '0.085', '0.0', '0', '0',
      PathJoinSubstitution([edu_robot_namespace, 'base_link']),
      PathJoinSubstitution([edu_robot_namespace, 'marker_camera'])
    ]    
  )

  # Pose Filtering and Controlling
  pose_controller_parameter_file = PathJoinSubstitution([
    FindPackageShare('edu_fleet'),
    'parameter',
    'qr_code_following',
    'pose_control.yaml'
  ])

  pose_controller = Node(
    package='edu_fleet',
    executable='pose_controller',
    name='pose_controller',
    namespace=edu_robot_namespace,
    parameters=[
      pose_controller_parameter_file,
      {'use_sim_time': use_sim_time},
      {'target_frame_id': PathJoinSubstitution([edu_robot_namespace, 'marker_camera'])} # todo: change to base_link
    ],
    remappings=[
      ('pose_feedback', 'marker/pose'),
      ('pose_target', 'target_pose'),
      ('twist_output', 'autonomous/cmd_vel')
    ],
    # prefix=['gdbserver localhost:3000'],
    output='screen'
  )

  # Publishing Target Pose
  publish_target_pose = ExecuteProcess(cmd=[
    'ros2',
    'topic',
    'pub',
    PathJoinSubstitution([edu_robot_namespace, 'target_pose']),
    'geometry_msgs/msg/PoseStamped', '{header: {frame_id: eduard/green/marker_camera}, pose: {position: {x: -1.0}}}'
  ])

  return LaunchDescription([
    edu_robot_namespace_arg,
    use_sim_time_arg,    
    camera_node,
    marker_pose_estimation,
    qr_code_deteciton,
    tf_camera_transform,
    pose_controller,
    publish_target_pose
  ])