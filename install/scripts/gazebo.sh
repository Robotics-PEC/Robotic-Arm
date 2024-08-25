#!/bin/bash
set -e

sudo apt-get -y update
sudo DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-ign-ros2-control \

# vi: ts=2 sw=2 et
