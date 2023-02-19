# edu_swarm

The edu_swarm_brain node is a Python-based ROS2 node that controls a swarm of three robots in a triangular formation, where each robot maintains a position relative to a common center point. 

## Function

The node subscribes to three topics (/swarm_1/odom, /swarm_2/odom, /swarm_3/odom) to receive the odometry information for each robot, and one topic (/cmd_vel) to receive velocity commands that control the position of the center point. The node calculates the desired positions of the robots based on the center point and their current positions, and publishes velocity commands on three topics (/swarm_1/cmd_vel, /swarm_2/cmd_vel, /swarm_3/cmd_vel) to control the movement of each robot. The node also includes position tolerance to ensure that the robots maintain their desired positions with a tolerance of 0.05m. The control logic for the robots is implemented in the odom callbacks, where each robot's velocity is set based on its current position and the desired position, so that it turns in the direction of its destination and **then** starts moving forward.

## Dependency

This node is developed with the [edu_simulation](https://github.com/EduArt-Robotik/edu_simulation) repository in mind.

## Running the node

```console
ros2 run edu_swarm edu_swarm_brain
```
