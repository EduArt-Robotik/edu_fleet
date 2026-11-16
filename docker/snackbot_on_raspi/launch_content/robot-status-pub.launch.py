import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
   # Get Robot Namespace
    robot_namespace = os.environ.get('EDU_ROBOT_NAMESPACE', 'eduard')
    if len(robot_namespace) == 0: robot_namespace = '/eduard/'                          # default value
    if robot_namespace[0] != '/': robot_namespace = '/' + robot_namespace               # ensure preceding "/"
    if robot_namespace[len(robot_namespace) - 1] != '/': robot_namespace += '/'         # ensure trailing "/"
    tf_prefix = robot_namespace[1:] if robot_namespace[0] == '/' else robot_namespace   # removes preceding "/" for tf_prefix

    # Debug prints
    print('use robot namespace = ', robot_namespace)
    print('use tf prefix = ', tf_prefix)

    status_topic_name = LaunchConfiguration("status_topic_name")
    status_topic_name_arg = DeclareLaunchArgument(
        'status_topic_name', default_value="status_topic"
    )

    status_node = Node(
        package='edu_fleet',
        executable='robot_status_pub_node',
        name='robot_status_publisher',
        namespace=robot_namespace,
        output='screen',
        parameters=[{ # change parameters like tf_frames here to apply tf prefix
            "topic_name": status_topic_name,
            'map_frame': tf_prefix + 'map',
            'base_frame': tf_prefix + 'base_footprint',
            'sensor_frame': tf_prefix + 'base_link',
        }]
    )

    ld = LaunchDescription()

    ld.add_action(status_topic_name_arg)
    ld.add_action(status_node)

    return ld
