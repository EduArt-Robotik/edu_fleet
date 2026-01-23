FROM eduartrobotik/eduart-perception:0.5.0

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

USER root

# Install Dependencies for Perception Package
RUN apt-get update \
    && apt-get install -y \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-aruco-opencv \
    ros-$ROS_DISTRO-aruco-opencv-msgs \
    ros-$ROS_DISTRO-laser-geometry \
    libcurl4-openssl-dev \
    libjsoncpp-dev \
    && apt-get clean

# work with user user
USER $USER

# building dependent packages first
## copy in needed software sources
COPY ./sick_lidar_localization /home/$USER/ros/src/sick_lidar_localization
COPY ./accerion-ros2/accerion_driver_msgs /home/$USER/ros/src/accerion_driver_msgs

USER root
RUN cp -f /home/$USER/ros/src/sick_lidar_localization/sick_lidar_localization_driver/package_ros2.xml /home/$USER/ros/src/sick_lidar_localization/sick_lidar_localization_driver/package.xml
RUN cp -f /home/$USER/ros/src/sick_lidar_localization/sick_lidar_localization_msgs/package_ros2.xml /home/$USER/ros/src/sick_lidar_localization/sick_lidar_localization_msgs/package.xml
USER $USER

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && cd /home/$USER/ros/ \
    && colcon build --packages-select accerion_driver_msgs sick_lidar_localization_driver sick_lidar_localization_msgs --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_VERSION=2

# building edu_fleet package
COPY ./edu_fleet /home/$USER/ros/src/edu_fleet
RUN echo "building edu_fleet" \
    && source install/setup.bash \
    && cd /home/$USER/ros/ \
    && colcon build --packages-select  edu_fleet --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
