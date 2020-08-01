FROM ros:melodic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python-vcstool

# ROS setting
RUN mkdir -p catkin_ws/src

RUN cd catkin_ws/src && \
    source /opt/ros/melodic/setup.bash && \
    catkin_init_workspace

COPY . /root/catkin_ws/src/repo

RUN cd /root/catkin_ws/src && \
    vcs import < repo/.rosinstall

RUN cd /root/catkin_ws && \
    rosdep update && \
    rosdep install -i -r -y --from-paths src && \
    source /opt/ros/melodic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

RUN echo 'source /opt/ros/melodic/setup.bash && source /root/catkin_ws/devel/setup.bash && exec "$@"' \
    > /root/ros_entrypoint.sh

WORKDIR /root/catkin_ws

ENTRYPOINT ["bash", "/root/ros_entrypoint.sh"]
