from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
  # Launch File Arguments
  robot_namespace = EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard")    
  use_pose_controller = LaunchConfiguration('use_pose_controller', default=True)
  use_pose_controller_arg = DeclareLaunchArgument(
    'use_pose_controller',
    default_value='True'
  )

  eduard_red = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('edu_swarm'),
        'launch',
        'eduard-fleet-drive-slave.launch.py'
      ]),
    ]),
    launch_arguments={
      'parameter_file_name': 'eduard-red-fleet-drive-slave.yaml',
      'use_pose_controller': use_pose_controller      
    }.items()
  )

  tf_publisher_cam_front = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
      '0.105', '0.04', '0.05', '0', '0', '0', # red
      PathJoinSubstitution([robot_namespace, 'base_link']),
      PathJoinSubstitution([robot_namespace, 'object_sensor', 'front'])
    ]
  )

  return LaunchDescription([
    use_pose_controller_arg,
    eduard_red,
    tf_publisher_cam_front
  ])

