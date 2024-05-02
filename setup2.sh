#!/bin/bash
sudo apt update
sudo apt install libudev-dev

arch=$(uname -i)
if [[ $arch == x86_64 ]]; then
  echo 'This is x86_64 Architecture'
  cd /workspaces/isaac_ros-dev/ && \
      sudo dpkg -i ftp.evocortex.com/libirimager-8.10.1-amd64.deb
elif [[ $arch == aarch64 ]]; then
  echo 'This is ARM Architecture'
  cd /workspaces/isaac_ros-dev/ && \
      sudo dpkg -i ftp.evocortex.com/libirimager-8.10.1-arm64.deb 
fi

cd /workspaces/isaac_ros-dev/ && \
    rosdep install -i -r --from-paths src --rosdistro humble -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv nvblox"

cd /workspaces/isaac_ros-dev && \
  colcon build --symlink-install && \
  source install/setup.bash