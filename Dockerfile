# Use the OSRF ROS Hubmle Desktop Full image as the base
FROM osrf/ros:humble-desktop-full

LABEL maintainer="Witty Wizard <shashankmarch27@gmail.com>"

# environment
ENV PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
ENV XDG_RUNTIME_DIR=/tmp/runtime-docker
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV TERM=xterm-256color
ENV DISPLAY=:20
ENV PATH="/home/user/bin:${PATH}"

# Set default shell during Docker image build to bash
SHELL ["/bin/bash", "-l", "-c"]

# Update and install additional packages if necessary
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    nano \
    xauth \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    locales \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Initialise system locale
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
RUN locale-gen en_US.UTF-8

# Create a new user named 'jazzer' with sudo privileges
RUN useradd -m jazzer && echo "jazzer:password" | chpasswd && adduser jazzer sudo

# Copy the entry point script into the container
COPY entrypoint.sh /usr/local/bin/entrypoint.sh

## Make sure the script is executable and accessible by 'jazzer'
RUN chmod +x /usr/local/bin/entrypoint.sh \
    && chown jazzer:jazzer /usr/local/bin/entrypoint.sh

# Switch to the 'jazzer' user
USER jazzer

# Set up a working directory
RUN mkdir -p /home/jazzer/workspace/
WORKDIR /home/jazzer/workspace

# Make sure the script is executable
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set the entry point to the script
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# (Optional) Set up an entrypoint or CMD
CMD ["bash"]