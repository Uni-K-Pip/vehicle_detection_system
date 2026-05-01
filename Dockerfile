# Reproducible build image for vehicle_detection_system.
#
# Build:
#   docker build -t vehicle_detection:dev .
# Run (from this repository root):
#   docker run --rm -it \
#     -v "$PWD":/workspace/vehicle_detection_system \
#     -w /workspace/vehicle_detection_system \
#     vehicle_detection:dev
#
# Then inside the container:
#   ./tools/run_demo.sh data/pcd/sample.pcd
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Keep the base ROS 2 libraries in sync with newly installed packages.
# Mixing an older desktop image with newer message packages can produce
# runtime symbol lookup errors in Fast-CDR / Fast-RTPS type support.
RUN apt-get update -qq \
    && apt-get dist-upgrade -y --no-install-recommends \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-jazzy-vision-msgs \
        ros-jazzy-tf2-sensor-msgs \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/vehicle_detection_system

# rosdep is preinstalled in osrf/ros:jazzy-desktop. Users mount their
# workspace at /workspace/vehicle_detection_system and run colcon there;
# nothing is copied at image build time so the image stays generic.

CMD ["/bin/bash"]
