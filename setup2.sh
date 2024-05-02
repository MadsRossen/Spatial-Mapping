#!/bin/bash
sudo apt update
sudo apt install libudev-dev
cd /workspaces/isaac_ros-dev/ && \
    sudo dpkg -i ftp.evocortex.com/libirimager-8.10.1-arm64.deb 

cd /workspaces/isaac_ros-dev/ && \
    rosdep install -i -r --from-paths src --rosdistro humble -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv nvblox"

cd /workspaces/isaac_ros-dev && \
  colcon build --symlink-install && \
  source install/setup.bash