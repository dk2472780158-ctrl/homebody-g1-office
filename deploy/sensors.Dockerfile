FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-dev python3-colcon-common-extensions \
    python3-numpy python3-scipy python3-matplotlib python3-websocket python3-cbor2 \
    libeigen3-dev libpcl-dev ros-humble-pcl-conversions \
    ros-humble-tf2-ros ros-humble-rclpy ros-humble-nav2-amcl \
    ros-humble-nav2-msgs ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY office/sensors/src /workspace/src
RUN bash -c 'source /opt/ros/humble/setup.bash && colcon build \
    --install-base /opt/homebody-sensor-install \
    --packages-up-to g1_fastlio_bridge --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3'
CMD ["bash", "/workspace/run-readonly-slam.sh"]
