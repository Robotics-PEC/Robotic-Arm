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

# Copy docker clean script
COPY install/docker/docker_clean.sh /docker_clean.sh

# Update and install additional packages if necessary
RUN DEBIAN_FRONTEND=noninteractive apt-get -y update && \
    apt upgrade -q -y && \
    DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
    locales \
    && \
    /docker_clean.sh

# Initialise system locale
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
RUN locale-gen en_US.UTF-8

COPY install/scripts/ros.sh /tmp/install/ros.sh
RUN /tmp/install/ros.sh && /docker_clean.sh

COPY install/scripts/gazebo.sh /tmp/install/gazebo.sh
RUN /tmp/install/gazebo.sh && /docker_clean.sh

# Create a new user named 'jazzer' with sudo privileges
RUN useradd -m jazzer && echo "jazzer:password" | chpasswd && usermod -aG sudo jazzer

# Make it so that no password is required for sudo commands
RUN echo "jazzer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the 'jazzer' user
USER jazzer

# create setting directory for gazebo
VOLUME /home/jazzer/.gz
RUN mkdir -p /home/jazzer/.gz

# create setting directory for jupyter
VOLUME /home/jazzer/.jupyter
RUN mkdir -p /home/jazzer/.jupyter

# create .X11-unix directory
VOLUME /tmp/.X11-unix
RUN mkdir -p /tmp/.X11-unix

# create .gnupg directory
VOLUME /home/jazzer/.gnupg
RUN mkdir -p /home/jazzer/.gnupg

# create .ssh directory
VOLUME /home/jazzer/.ssh
RUN mkdir -p /home/jazzer/.ssh

# create .gitconfig file
VOLUME /home/jazzer/.gitconfig

# Set up a working directory
RUN mkdir -p /home/jazzer/workspace/
WORKDIR /home/jazzer/workspace

USER root
# Copy the entry point script into the container
COPY install/docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Set the entry point to the script
CMD ["/bin/bash"]
ENTRYPOINT ["/entrypoint.sh"]