import rclpy
import pytest
import unittest
import threading

from math import pi
from time import time, sleep

import rclpy.duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy.time
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

@pytest.mark.launch_test
def generate_test_description():
  node_under_test = Node(
    package='edu_fleet',
    executable='fleet_localization_node',
    output='screen',
    remappings=[
      ('eduard/imu', 'component_test/fleet_localization/imu'),
      ('eduard/odometry', 'component_test/fleet_localization/odometry'),
      ('eduard/pose', 'component_test/fleet_localization/pose')
    ]
  )

  return LaunchDescription([
    node_under_test,
    launch_testing.actions.ReadyToTest()
  ])

class ComponentTesFleetLocalization(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    # Initialize the ROS context for the test node
    rclpy.init()

  @classmethod
  def tearDownClass(cls):
    # Shutdown the ROS context
    rclpy.shutdown()

  def setUp(self):
    namespace = 'component_test/fleet_localization'
    self.node = rclpy.create_node('component_fleet_localization')
    self.pub_imu = self.node.create_publisher(Imu, namespace + '/imu', 2)
    self.pub_odometry = self.node.create_publisher(Odometry, namespace + '/odometry', 2)
    self.pub_pose = self.node.create_publisher(PoseStamped, namespace + '/pose', 2)
    self.client_reset = self.node.create_client(Trigger, '/reset')

    self.is_running = True
    self.spinner = threading.Thread(target=self.execute_ros, daemon=True)
    self.spinner.start()

  def tearDown(self) -> None:
    super().tearDown()
    self.is_running = False
    self.spinner.join()

  def execute_ros(self) -> None:
    while self.is_running is True:
      rclpy.spin_once(self.node)
      sleep(0.005)

  def test_acceleration_x_direction(self, launch_service, proc_output):
    # reset fleet localization
    assert self.client_reset.service_is_ready() is True
    future = self.client_reset.call_async(Trigger.Request())
    future.result()
    
    # sending measurement
    rate = self.node.create_rate(40.0, self.node.get_clock())
    stamp_start = self.node.get_clock().now()
    duration = rclpy.time.Duration(seconds=3)

    imu_msg = Imu()
    imu_msg.header.frame_id = 'imu'
    
    imu_msg.linear_acceleration.x = 1.0
    imu_msg.linear_acceleration_covariance[0] = 0.2 * 0.2
    imu_msg.linear_acceleration_covariance[4] = 0.2 * 0.2
    
    imu_msg.angular_velocity.z = 0.0
    imu_msg.angular_velocity_covariance[8] = 0.087266463 * 0.087266463 # 5° std dev

    print('sending imu messages with acceleration x = ', imu_msg.linear_acceleration.x)

    while rclpy.ok() and (self.node.get_clock().now() - stamp_start) < duration:
      imu_msg.header.stamp = self.node.get_clock().now().to_msg()
      self.pub_imu.publish(imu_msg)

      rate.sleep()

  def test_stop_acceleration_x_direction(self, launch_service, proc_output):
    # reset fleet localization
    assert self.client_reset.service_is_ready() is True
    future = self.client_reset.call_async(Trigger.Request())
    future.result()

    # sending measurement
    rate = self.node.create_rate(40.0, self.node.get_clock())
    stamp_start = self.node.get_clock().now()
    duration = rclpy.time.Duration(seconds=3)

    imu_msg = Imu()
    imu_msg.header.frame_id = 'imu'
    
    imu_msg.linear_acceleration.x = 0.0
    imu_msg.linear_acceleration_covariance[0] = 0.2 * 0.2
    imu_msg.linear_acceleration_covariance[4] = 0.2 * 0.2
    
    imu_msg.angular_velocity.z = 0.0
    imu_msg.angular_velocity_covariance[8] = 0.087266463 * 0.087266463 # 5° std dev

    print('sending imu messages with acceleration x = ', imu_msg.linear_acceleration.x)

    while rclpy.ok() and (self.node.get_clock().now() - stamp_start) < duration:
      imu_msg.header.stamp = self.node.get_clock().now().to_msg()
      self.pub_imu.publish(imu_msg)

      rate.sleep()

  def test_drive_circle(self, launch_service, proc_output):
    # reset fleet localization
    assert self.client_reset.service_is_ready() is True
    future = self.client_reset.call_async(Trigger.Request())
    future.result()

    # sending measurement    
    rate = self.node.create_rate(40.0, self.node.get_clock())
    stamp_start = self.node.get_clock().now()
    duration = rclpy.time.Duration(seconds=10)

    odometry_msg = Odometry()
    odometry_msg.header.frame_id = 'map'
    odometry_msg.child_frame_id = 'eduard/base_link'
    
    odometry_msg.twist.twist.linear.x = 1.0
    odometry_msg.twist.covariance[0] = 0.2 * 0.2
    odometry_msg.twist.covariance[7] = 0.2 * 0.2
    
    odometry_msg.twist.twist.angular.z = (2.0 * pi) / 10.0 # one complete circle in 10 seconds
    odometry_msg.twist.covariance[35] = 0.087266463 * 0.087266463 # 5° std dev

    print('sending odometry messages with velocity x = ', odometry_msg.twist.twist.linear.x)
    print('and yaw rate = ', odometry_msg.twist.twist.angular.z)

    while rclpy.ok() and (self.node.get_clock().now() - stamp_start) < duration:
      odometry_msg.header.stamp = self.node.get_clock().now().to_msg()
      self.pub_odometry.publish(odometry_msg)

      rate.sleep()

  def test_drive_line_imu_and_odometry(self, launch_service, proc_output):
    print('test_drive_line_imu_and_odometry')

    # reset fleet localization
    assert self.client_reset.service_is_ready() is True
    future = self.client_reset.call_async(Trigger.Request())
    future.result()

    # sending measurement 
    rate = self.node.create_rate(40.0, self.node.get_clock())
    stamp_start = self.node.get_clock().now()
    duration = rclpy.time.Duration(seconds=10)

    # prepare imu message
    imu_msg = Imu()
    imu_msg.header.frame_id = 'imu'
    
    imu_msg.linear_acceleration.x = 0.0
    imu_msg.linear_acceleration_covariance[0] = 0.2 * 0.2
    imu_msg.linear_acceleration_covariance[4] = 0.2 * 0.2
    
    imu_msg.angular_velocity.z = 0.0
    imu_msg.angular_velocity_covariance[8] = 0.087266463 * 0.087266463 # 5° std dev

    # prepare odometry message
    odometry_msg = Odometry()
    odometry_msg.header.frame_id = 'map'
    odometry_msg.child_frame_id = 'eduard/base_link'
    
    odometry_msg.twist.twist.linear.x = 0.0
    odometry_msg.twist.covariance[0] = 0.2 * 0.2
    odometry_msg.twist.covariance[7] = 0.2 * 0.2
    
    odometry_msg.twist.covariance[35] = 0.087266463 * 0.087266463 # 5° std dev

    # measurement
    acceleration = 1.0
    velocity = 0.0
    last_stamp = self.node.get_clock().now()

    while rclpy.ok() and (self.node.get_clock().now() - stamp_start) < duration:
      now = self.node.get_clock().now()
      dt = (now - last_stamp).nanoseconds / 1000000000.0
      print('dt = ', dt)

      velocity += acceleration * dt

      imu_msg.header.stamp = now.to_msg()
      imu_msg.linear_acceleration.x = acceleration
      self.pub_imu.publish(imu_msg)

      odometry_msg.header.stamp = now.to_msg()
      odometry_msg.twist.twist.linear.x = velocity
      self.pub_odometry.publish(odometry_msg)

      last_stamp = now
      rate.sleep()