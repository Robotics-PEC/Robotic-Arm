# Use the OSRF ROS Jazzy Desktop Full image as the base
FROM osrf/ros:jazzy-desktop-full

# Update and install additional packages if necessary
RUN apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y\
    ros-jazzy-urdf-tutorial \
    && rm -rf /var/lib/apt/lists/*

# Create a new user named 'jazzy' with sudo privileges
RUN useradd -m jazzer && echo "jazzer:password" | chpasswd && adduser jazzer sudo

# Switch to the 'jazzer' user
USER jazzer

RUN mkdir /home/jazzer/src

# Set up a working directory
WORKDIR /home/jazzer

# (Optional) Set up an entrypoint or CMD
CMD ["bash"]