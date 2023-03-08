import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_path = get_package_share_path('edu_swarm')
    parameter_file = os.path.join(
      package_path,
      'parameter',
      'eduard-green-fleet-drive.yaml'
    )

    pose_controller = Node(
      package='edu_swarm',
      executable='pose_controller',
      name='pose_controller',
      namespace='eduard/green',
      parameters=[parameter_file],
      remappings=[
        ('pose_feedback', 'object/pose'),
        ('twist_output', 'cmd_vel')
      ],
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    tf_publisher_cam_front = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['0.17', '0', '0.05', '0', '0', '0', 'eduard/green/base_link', 'eduard/green/object_sensor/front']
    )
    tf_publisher_cam_left = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['0', '0.092', '0.05', '1.57', '0', '0', 'eduard/green/base_link', 'eduard/green/object_sensor/left']
    )
    tf_publisher_cam_right = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['0', '-0.092', '0.05', '-1.57', '0', '0', 'eduard/green/base_link', 'eduard/green/object_sensor/right']
    )
    tf_publisher_cam_rear = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['-0.17', '0', '0.05', '3.141592654', '0', '0', 'eduard/green/base_link', 'eduard/green/object_sensor/rear']
    )      

    return LaunchDescription([
      pose_controller,
      tf_publisher_cam_front,
      tf_publisher_cam_left,
      tf_publisher_cam_right,
      tf_publisher_cam_rear
    ])
    