from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
  # Launch File Arguments
  use_pose_controller = LaunchConfiguration('use_pose_controller', default=True)
  use_pose_controller_arg = DeclareLaunchArgument(
    'use_pose_controller',
    default_value='True'
  )

  eduard_green = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('edu_swarm'),
        'launch',
        'eduard-fleet-drive-slave.launch.py'
      ]),
    ]),
    launch_arguments={
      'parameter_file_name': 'eduard-green-fleet-drive-slave.yaml',
      'robot_namespace': 'eduard/green',
      'use_pose_controller': use_pose_controller
    }.items()
  )

  return LaunchDescription([
    use_pose_controller_arg,
    eduard_green,
  ])