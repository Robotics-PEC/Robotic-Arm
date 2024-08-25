#!/bin/bash
set -e

ROS_VERSION="humble"

sudo apt-get -y update
sudo DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
  ament-cmake \
  python3-colcon-common-extensions \
  python3-colcon-ros \
  python3-rosdep \
  python3-vcstool \
  software-properties-common \
  python3-pip \
  nano \
  xauth \
  ros-${ROS_VERSION}-joint-state-publisher-gui \
  ros-${ROS_VERSION}-moveit \
  ros-${ROS_VERSION}-ros2-controllers \
  ros-${ROS_VERSION}-controller-manager \
  ros-${ROS_VERSION}-actuator-msgs \
  ros-${ROS_VERSION}-compressed-image-transport \
  ros-${ROS_VERSION}-cyclonedds \
  ros-${ROS_VERSION}-desktop \
  ros-${ROS_VERSION}-gps-msgs \
  ros-${ROS_VERSION}-rmw-cyclonedds-cpp \
  ros-${ROS_VERSION}-rqt-tf-tree \
  ros-${ROS_VERSION}-topic-tools \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_VERSION}-ign-ros2-control \
  locales \
  wget \

sudo pip install cyclonedds pycdr2

# vi: ts=2 sw=2 et
