# Docker alternative (native install is primary). On a Jetson use an L4T-based ROS image, e.g.
#   docker build --build-arg BASE=dustynv/ros:humble-ros-base-l4t-r36.2.0 -t eyesteer deploy/docker
# On x86 (CI / laptop):  docker build -t eyesteer -f deploy/docker/Dockerfile .
ARG BASE=ros:humble-ros-base
FROM ${BASE}
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-twist-mux \
      ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-rosbridge-suite ros-humble-xacro \
      ros-humble-rmw-cyclonedds-cpp libserial-dev python3-pip python3-scipy python3-serial git openssl \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir "numpy<1.25" pylsl pyyaml "mujoco<3.12" cbor2
WORKDIR /eyesteer
COPY . /eyesteer
RUN bash deploy/build_deps_from_source.sh --diffdrive-only \
 && bash -c "source /opt/ros/humble/setup.bash && source ~/eyesteer_deps_ws/install/setup.bash && cd ros2_ws && colcon build --symlink-install"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CMD ["bash", "-c", "source deploy/env.sh && ros2 launch eyesteer_bringup sim.launch.py"]
