FROM ubuntu:20.04
LABEL name = "Mads Rossen"
LABEL mail = "madsrossen@me.com"
LABEL version = "0.1"

# Distribution of ROS2
ARG distro="humble"

# Install necessary software for the installation of ROS2
RUN apt-get update && apt-get install -y \ 
                      locales \
                      curl \
                      gnupg2 \
                      lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

# Set the locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Add key
RUN curl http://repo.ros2.org/repos.key | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http:packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt update

# Specify the distribution of ROS2
ENV ROS_DISTRO=$distro
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_PYTHON_VERSION=3

# Install ROS2
RUN apt install -y ros-$ROS_DISTRO-desktop \
                python3-colcon-common-extensions \
                python3-rosdep \
                python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*rm 

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup scripts
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Set the entry point
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]