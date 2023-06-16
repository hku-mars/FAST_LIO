FROM ros:noetic-perception

RUN apt update && apt install -y git

# Install Livox SDK
RUN cd /tmp && \
    git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    cd build && \
    cmake .. && \
    make -j2 && \
    make install && \
    rm -rf /tmp/Livox-SDK

# Download Livox ROS Driver
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git app/ws_fast_lio/src/livox_ros_driver

# Install Livox SDK2
RUN cd /tmp && \
    git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j2 && \
    make install && \
    rm -rf /tmp/Livox-SDK2

# Install Livox ROS Driver 2
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git app/ws_fast_lio/src/livox_ros_driver2 && \
    cd /app/ws_fast_lio/ && \
    . /opt/ros/noetic/setup.sh && \
    cp src/livox_ros_driver2/package_ROS1.xml src/livox_ros_driver2/package.xml && \
    catkin_make -j2 -DROS_EDITION=ROS1

# COPY FAST_LIO
COPY . /app/ws_fast_lio/src/FAST_LIO

RUN cd /app/ws_fast_lio/ && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make -j1 -DROS_EDITION=ROS1
