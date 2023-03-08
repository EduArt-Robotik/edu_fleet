from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
  eduard_green = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('edu_swarm'),
        'launch/',
        'eduard-green-fleet-drive-slave.py'
      ]),
    ]),
  )

  eduard_blue = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('edu_swarm'),
        'launch/',
        'eduard-blue-fleet-drive-slave.py'
      ]),
    ]),
  )

  eduard_red = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('edu_swarm'),
        'launch/'
        'eduard-fleet-drive-master.py'
      ]),
    ])
  )

  return LaunchDescription([
    eduard_red,
    eduard_green,
    eduard_blue
  ])

