# edu_swarm

This package provides control software for a fleet application in which three robots drive together in a formation. The package is based on the paper: **Hierarchical Multi-Robot Fleet Architecture with a
Kinematics-adaptive Drive System** .

Unfortunately the documentation is still missing. If you have any questions, please contact [Christian Wendt](mailto:christian.wendt@eduart-robotik.com).

# Building Dependencies

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-aruco-opencv-msgs
```


# Robot Fleet Ip Configuration

## Eduard Blue

|----|-----------|-------------|
| IP | Component | Description |
|----|-----------|-------------|
|192.168.0.100 | (eno2) |
|192.168.0.17 | Level 2 Start Address (eno2) |

## Eduard Red

|----|-----------|-------------|
| IP | Component | Description |
|----|-----------|-------------|
|192.168.0.101| Eduard Red (eno2)
|192.168.0.40 | Level 2 Start Address (eno2) |
|192.168.3.110| Oak D Cam |

## Eduard Green

|----|-----------|-------------|
| IP | Component | Description |
|----|-----------|-------------|
|192.168.0.102| Eduard Red (eno2)
|192.168.0.60 | Level 2 Start Address (eno2) |
|192.168.0.111| Oak D Cam |
