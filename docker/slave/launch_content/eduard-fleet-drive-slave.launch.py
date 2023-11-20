import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch File Arguments
    robot_namespace = EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard")
    use_pose_controller = LaunchConfiguration('use_pose_controller', default=True)

    robot_namespace_arg = DeclareLaunchArgument(
      'robot_namespace',
      default_value='robot'
    )
    parameter_file_name_arg = DeclareLaunchArgument(
      'parameter_file_name',
      default_value='eduard-green-fleet-drive-slave.yaml'
    )
    use_pose_controller_arg = DeclareLaunchArgument(
      'use_pose_controller',
      default_value='True'
    )

    # Pose Controller Node
    # robot_namespace = str('eduard/green')
    parameter_file = PathJoinSubstitution([
      './',
      'eduard-fleet-drive-slave.yaml'
    ])

    pose_controller = Node(
      package='edu_swarm',
      executable='pose_controller',
      name='pose_controller',
      namespace=robot_namespace,
      parameters=[parameter_file],
      remappings=[
        ('pose_feedback', 'object/pose'),
        ('twist_output', 'pose_controller/cmd_vel')
      ],
      condition=IfCondition(use_pose_controller),
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    tf_publisher_cam_front = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.17', '0.04', '0.05', '0', '0', '0',
        PathJoinSubstitution([robot_namespace, 'base_link']),
        PathJoinSubstitution([robot_namespace, 'object_sensor', 'front'])
      ]
    )
    tf_publisher_cam_left = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0', '0.092', '0.05', '1.57', '0', '0',
         PathJoinSubstitution([robot_namespace, 'base_link']),
         PathJoinSubstitution([robot_namespace, 'object_sensor', 'left'])
      ]
    )
    tf_publisher_cam_right = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0', '-0.092', '0.05', '-1.57', '0', '0',
        PathJoinSubstitution([robot_namespace, 'base_link']),
        PathJoinSubstitution([robot_namespace, 'object_sensor', 'right'])
      ]
    )
    tf_publisher_cam_rear = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '-0.17', '0', '0.05', '3.141592654', '0', '0',
        PathJoinSubstitution([robot_namespace, 'base_link']),
        PathJoinSubstitution([robot_namespace, 'object_sensor', 'rear'])
      ]
    )

    # Twist Accumulation
    twist_accumulator = Node(
      package='edu_swarm',
      executable='twist_accumulator',
      name='twist_accumulator',
      namespace=robot_namespace,
      parameters=[parameter_file],
      remappings=[
        ('twist/input_0', 'fleet_control/cmd_vel'),
        ('twist/input_1', 'pose_controller/cmd_vel'),
        ('twist/output', 'fleet/cmd_vel')
      ],
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    return LaunchDescription([
      robot_namespace_arg,
      parameter_file_name_arg,
      use_pose_controller_arg,
      pose_controller,
      tf_publisher_cam_front,
      tf_publisher_cam_left,
      tf_publisher_cam_right,
      tf_publisher_cam_rear,
      twist_accumulator
    ])
    