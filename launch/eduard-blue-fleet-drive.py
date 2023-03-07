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
      'eduard-blue-fleet-drive.yaml'
    )

    pose_controller = Node(
      package='edu_swarm',
      executable='pose_controller',
      name='pose_controller',
      namespace='eduard/blue',
      parameters=[parameter_file],
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    return LaunchDescription([
      pose_controller
    ])
    