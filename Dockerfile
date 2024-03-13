FROM nvcr.io/nvidia/l4t-jetpack:r35.3.1
# FROM osrf/ros:humble-desktop-full
# FROM stereolabs/zedbot:zed-ros2-wrapper_l4t35_1_humble_
LABEL name = "Mads Rossen"
LABEL mail = "madsrossen@me.com"
LABEL version = "0.2"

# Distribution of ROS2
ARG DISTRO="humble"

# Install the ZED SDK
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=0
ARG ZED_SDK_PATCH=5
ARG L4T_MAJOR=35
ARG L4T_MINOR=3

RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
    apt-get update -y || true && \
    wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
    https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
    chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
    rm -rf /usr/local/zed/resources/* && \
    rm -rf ZED_SDK_Linux_JP.run && \
    rm -rf /var/lib/apt/lists/*

RUN python3 /usr/local/zed/get_python_api.py


# Install necessary software for the installation of ROS2
RUN apt-get update && apt-get install -y \ 
                      locales \
                      curl \
                      gnupg2 \
                      software-properties-common \
                      lsb-release

# Specify the distribution of ROS2s
ENV ROS_DISTRO=$DISTRO
ENV ROS_PYTHON_VERSION=3

# Add key to ensure the installation of ROS2 pkgs
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt-get update
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"
RUN tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get upgrade -y

# Setup scripts
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Set the entry point
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]