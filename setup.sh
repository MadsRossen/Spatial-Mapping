#!/bin/bash
#
# This file contain the software to setup 
# the environment for the Isaac ROS development 
# and the ZEd camera for the Jetsone Orin

# Host System Setup

sudo systemctl daemon-reload && sudo systemctl restart docker

sudo apt-get install git-lfs
git lfs install --skip-repo

sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608

echo -e "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf

cd ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/

echo "installing the dependencies for NvBlox"

cd ${ISAAC_ROS_WS}/src/ && \
  git submodule update --init --recursive 

cd ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/ && git lfs pull && \   
  git submodule update --init --recursive 

cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh ${ISAAC_ROS_WS}
# Installing the Dependencies
# (Add the commands from the text here)

# Example with ZED Live Data
# (Add the commands from the text here)

