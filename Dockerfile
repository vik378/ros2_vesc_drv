FROM ros:foxy-ros-base

# install pip3 and python deps
RUN apt update
RUN apt install --assume-yes python3-pip
RUN pip3 install pyserial

# clone ros package repo
ENV ROS2_WS  /home/ros2_ws
RUN mkdir -p ${ROS2_WS}/src/ros2_vesc_drv
COPY ./ ${ROS2_WS}/src/ros2_vesc_drv

# build repo
RUN cd ${ROS2_WS} \
    && . /opt/ros/foxy/setup.sh \
    && colcon build \
    && . ${ROS2_WS}/install/setup.sh

WORKDIR ${ROS2_WS}
RUN echo "source install/setup.bash" >> /opt/ros/foxy/setup.bash
