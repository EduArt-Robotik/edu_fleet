import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import Odometry


class EduSwarmBrain(Node):

    def __init__(self):
        super().__init__('edu_swarm_brain')

        # Initialize subscribers for odom
        self.sub1 = self.create_subscription(Odometry, 'swarm_1/odom', self.odom_callback1, 10)
        self.sub2 = self.create_subscription(Odometry, 'swarm_2/odom', self.odom_callback2, 10)
        self.sub3 = self.create_subscription(Odometry, 'swarm_3/odom', self.odom_callback3, 10)

        # Initialize subscriber for twist
        self.sub_twist = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

        # Initialize publishers for twist
        self.pub1 = self.create_publisher(Twist, 'swarm_1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, 'swarm_2/cmd_vel', 10)
        self.pub3 = self.create_publisher(Twist, 'swarm_3/cmd_vel', 10)

        # Set the initial position of the center point
        self.center_point = Point(x=0.0, y=0.0, z=0.0)

        # Set the desired positions for the robots relative to the center point
        self.delta_x = 5.0
        self.delta_y = 2.5
        self.desired_pos1 = Point(x=self.center_point.x + self.delta_x, y=self.center_point.y + self.delta_y, z=self.center_point.z)
        self.desired_pos2 = Point(x=self.center_point.x + self.delta_x, y=self.center_point.y - self.delta_y, z=self.center_point.z)
        self.desired_pos3 = Point(x=self.center_point.x - self.delta_x, y=self.center_point.y, z=self.center_point.z)

    def odom_callback1(self, msg):
        # Update the position of Swarm Agent 1
        self.pos1 = msg.pose.pose.position

        # Calculate the distance between the current position and the desired position for Swarm Agent 1
        dx = self.desired_pos1.x - self.pos1.x
        dy = self.desired_pos1.y - self.pos1.y
        dist = ((dx) ** 2 + (dy) ** 2) ** 0.5

        # Calculate the desired velocity for Swarm Agent 1 based on its current position and the desired position
        if dist > 0.05:
            # Calculate the desired heading for Swarm Agent 1 based on its current position and the desired position
            angle = math.atan2(dy, dx)

            # Calculate the difference between the current and desired headings
            dangle = angle - self.pos1.z

            # If the difference between the current and desired headings is greater than pi, wrap it around
            if dangle > math.pi:
                dangle -= 2 * math.pi
            elif dangle < -math.pi:
                dangle += 2 * math.pi

            # Set the angular velocity component to the difference between the desired heading and the current heading
            # (multiplied by a scaling factor of 0.5 to reduce the angular velocity)
            w1 = Vector3(x=0.0, y=0.0, z=dangle * 0.5)

            # If the Swarm Agent is not facing its destination, set its linear velocity to zero to make it turn in place
            if abs(dangle) > 0.1:
                v1 = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                # If the Swarm Agent is facing its destination, set its linear velocity to a fixed value of 0.2 m/s
                v1 = Vector3(x=0.2, y=0.0, z=0.0)
        else:
            # If Swarm Agent 1 has reached its desired position, stop moving
            v1 = Vector3(x=0.0, y=0.0, z=0.0)
            w1 = Vector3(x=0.0, y=0.0, z=0.0)

        # Publish the desired velocity for Swarm Agent 1
        twist1 = Twist(linear=v1, angular=w1)
        self.pub1.publish(twist1)

    def odom_callback2(self, msg):
        # Update the position of Swarm Agent 2
        self.pos2 = msg.pose.pose.position

        # Calculate the distance between the current position and the desired position for Swarm Agent 2
        dx = self.desired_pos2.x - self.pos2.x
        dy = self.desired_pos2.y - self.pos2.y
        dist = ((dx) ** 2 + (dy) ** 2) ** 0.5

        # Calculate the desired velocity for Swarm Agent 2 based on its current position and the desired position
        if dist > 0.05:
            # Calculate the desired heading for Swarm Agent 2 based on its current position and the desired position
            angle = math.atan2(dy, dx)

            # Calculate the difference between the current and desired headings
            dangle = angle - self.pos2.z

            # If the difference between the current and desired headings is greater than pi, wrap it around
            if dangle > math.pi:
                dangle -= 2 * math.pi
            elif dangle < -math.pi:
                dangle += 2 * math.pi

            # Set the angular velocity component to the difference between the desired heading and the current heading
            # (multiplied by a scaling factor of 0.5 to reduce the angular velocity)
            w2 = Vector3(x=0.0, y=0.0, z=dangle * 0.5)

            # If the Swarm Agent is not facing its destination, set its linear velocity to zero to make it turn in place
            if abs(dangle) > 0.1:
                v2 = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                # If the Swarm Agent is facing its destination, set its linear velocity to a fixed value of 0.2 m/s
                v2 = Vector3(x=0.2, y=0.0, z=0.0)
        else:
            # If Swarm Agent 2 has reached its desired position, stop moving
            v2 = Vector3(x=0.0, y=0.0, z=0.0)
            w2 = Vector3(x=0.0, y=0.0, z=0.0)

        # Publish the desired velocity for Swarm Agent 2
        twist2 = Twist(linear=v2, angular=w2)
        self.pub2.publish(twist2)


    def odom_callback3(self, msg):
        # Update the position of Swarm Agent 3
        self.pos3 = msg.pose.pose.position

        # Calculate the distance between the current position and the desired position for Swarm Agent 3
        dx = self.desired_pos3.x - self.pos3.x
        dy = self.desired_pos3.y - self.pos3.y
        dist_x = abs(dx)
        dist_y = abs(dy)

        # Check if Swarm Agent 3 has reached its desired position in the x and y directions
        if dist_x > 0.05 or dist_y > 0.05:
            # Calculate the desired heading for Swarm Agent 3 based on its current position and the desired position
            angle = math.atan2(dy, dx)

            # Calculate the difference between the current and desired headings
            dangle = angle - self.pos3.z

            # If the difference between the current and desired headings is greater than pi, wrap it around
            if dangle > math.pi:
                dangle -= 2 * math.pi
            elif dangle < -math.pi:
                dangle += 2 * math.pi

            # Set the angular velocity component to the difference between the desired heading and the current heading
            # (multiplied by a scaling factor of 0.5 to reduce the angular velocity)
            w3 = Vector3(x=0.0, y=0.0, z=dangle * 0.5)

            # If the Swarm Agent is not facing its destination, set its linear velocity to zero to make it turn in place
            if abs(dangle) > 0.1:
                v3 = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                # If the Swarm Agent is facing its destination, set its linear velocity to a fixed value of 0.2 m/s
                v3 = Vector3(x=0.2, y=0.0, z=0.0)
        else:
            # If Swarm Agent 3 has reached its desired position, stop moving
            v3 = Vector3(x=0.0, y=0.0, z=0.0)
            w3 = Vector3(x=0.0, y=0.0, z=0.0)

        # Publish the desired velocity for Swarm Agent 3
        twist3 = Twist(linear=v3, angular=w3)
        self.pub3.publish(twist3)

    def twist_callback(self, msg):
        # Update the position of the center point based on the incoming twist message
        self.center_point.x += msg.linear.x
        self.center_point.y += msg.linear.y

        # Update the desired positions for the robots based on the new center point
        self.desired_pos1 = Point(x=self.center_point.x + self.delta_x, y=self.center_point.y + self.delta_y, z=self.center_point.z)
        self.desired_pos2 = Point(x=self.center_point.x + self.delta_x, y=self.center_point.y - self.delta_y, z=self.center_point.z)
        self.desired_pos3 = Point(x=self.center_point.x - self.delta_x, y=self.center_point.y, z=self.center_point.z)


def main(args=None):
    rclpy.init(args=args)
    edu_swarm_brain = EduSwarmBrain()
    rclpy.spin(edu_swarm_brain)
    edu_swarm_brain.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
