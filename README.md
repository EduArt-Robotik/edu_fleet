# edu_fleet

This repository contains a collection of software for robot tracking and navigation for Sick and Triton sensors.

## Fleet

will be updated soon

## Sick / Triton Navigation

> **Note:** Sick and Triton navigation packages will only be build if **sick_lidar_localization_msgs and accerion_driver_msgs** are available.

For more documentation see [here](documentation/sick/README.md).

## Building Package

Please ensure that the following dependencies are installed:

* geometry_msgs
* nav_msgs
* sensor_msgs
* visualization_msgs
* laser_geometry
* aruco_opencv_msgs
* diagnostic_msgs
* diagnostic_updater

And following EduArt packages:

* [edu_robot](https://github.com/EduArt-Robotik/edu_robot)
* [edu_perception](https://github.com/EduArt-Robotik/edu_perception)

The package could be build using following command:

```bash
colcon build --symlink-install --packages-select edu_fleet --event-handlers console_direct+
```

