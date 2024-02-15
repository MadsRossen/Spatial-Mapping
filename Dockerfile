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
                      software-properties-common \
                      lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

# Set the locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

RUN apt-get update
RUN add-apt-repository universe
RUN apt-get update --fix-missing
RUN apt-get upgrade

# Specify the distribution of ROS2s
ENV ROS_DISTRO=$distro
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_PYTHON_VERSION=3

# Add key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt-get update
RUN tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update

# Install ROS2
RUN apt-get install -y ros-$ROS_DISTRO-desktop \
                python3-colcon-common-extensions \
                python3-rosdep \
                python3-argcomplete \
                ros-dev-tools

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup scripts
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Set the entry point
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]