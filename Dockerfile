# Use the OSRF ROS Jazzy Desktop Full image as the base
FROM osrf/ros:jazzy-desktop-full

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
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-slam-toolbox \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Create a new user named 'jazzy' with sudo privileges
RUN useradd -m jazzer && echo "jazzer:password" | chpasswd && adduser jazzer sudo

# Switch to the 'jazzer' user
USER jazzer

# Set up a working directory
RUN mkdir -p /home/jazzer/workspace/src
WORKDIR /home/jazzer/workspace

# (Optional) Set up an entrypoint or CMD
CMD ["bash"]