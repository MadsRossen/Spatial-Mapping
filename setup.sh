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

# Installing the Dependencies
# (Add the commands from the text here)

# Example with ZED Live Data
# (Add the commands from the text here)

