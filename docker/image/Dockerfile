FROM eduartrobotik/eduart-perception:0.4.1

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

# copy in needed software sources
COPY ./edu_swarm /home/$USER/ros/src/edu_fleet
COPY ./sick_lidar_localization /home/$USER/ros/src/sick_lidar_localization
USER root
RUN cp -f /home/$USER/ros/src/sick_lidar_localization/package_ros2.xml /home/$USER/ros/src/sick_lidar_localization/package.xml
USER $USER

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --packages-select sick_lidar_localization --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_VERSION=2

RUN echo "building edu_fleet" \
    && source install/setup.bash \
    && colcon build --packages-select edu_fleet --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
