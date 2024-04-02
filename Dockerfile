FROM nvcr.io/nvidia/l4t-jetpack:r36.2.0
FROM osrf/ros:humble-desktop-full
FROM stereolabs/zedbot:zed-ros2-wrapper_l4t35_1_humble_
LABEL NAME = "Mads Rossen"
LABEL MAIL = "madsrossen@me.com"
LABEL VERSION = "0.3"

# Install necessary software for the installation of ROS2
RUN apt-get update && apt-get install -y \
                      locales \
                      curl \
                      gnupg2 \
                      software-properties-common \
                      lsb-release

# Setup scripts
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Add key to ensure the installation of ROS2 pkgs
RUN apt-get update && apt-get upgrade -y