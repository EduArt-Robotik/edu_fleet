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
  line_controller = Node(
    package='edu_fleet',
    executable='line_controller_node',
    name='line_controller',
    # parameters=[parameter_file],
    remappings=[
      ('out/velocity', 'out/velocity'),
      ('in/line_detection', '/localizationcontroller/out/line_measurement_message_0404')
    ],
    namespace=edu_robot_namespace,
    # prefix=['gdbserver localhost:3000'],
    output='screen'    
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    line_controller
  ])
    