FROM eduartrobotik/eduart-robot:0.5.0

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
    && apt-get clean

# work with user user
USER $USER

# copy in needed software sources
COPY ./edu_swarm /home/$USER/ros/src/edu_swarm

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
