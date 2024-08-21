
# Universal Robotics - Robotic Arm

## Overview
This project provides a ROS (Robot Operating System) package designed for controlling a Universal Robotics robotic arm.

## Prerequisits

- Operating System: Ubuntu 22.04 LTS (Recommended)
- ROS 2 Distribution: Humble
- Docker: Installed and configured for running ROS 2 applications
- NVIDIA Drivers: Ensure you have the latest drivers installed if using GPU acceleration.

> [!NOTE]  
> If you are using Docker, installing ROS 2 separately is not required.

## Installation
> [!IMPORTANT]  
> ROS 2 requires the universe repository, which is only available on Ubuntu or Ubuntu derivatives of Linux. If you are using Arch or other non-Ubuntu distributions, you must use Docker.

### Installation for Linux
#### 1. Install Ubuntu 22.04 LTS

If you haven't already installed Ubuntu, follow these steps:

1. Download the Ubuntu 22.04 LTS ISO from the [official website](https://ubuntu.com/).
2. Create a bootable USB stick using tools like Rufus (for Windows) or Startup Disk Creator (for Ubuntu).
3. Boot from the USB stick and follow the on-screen instructions to install Ubuntu.

#### 2. Install ROS 2 Humble 
> [!IMPORTANT]  
> Cross Check these installtion commands from the official ROS2 installation guide

Follow the official ROS 2 [installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for Ubuntu:

1. Set up your sources:
    ```bash
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    ```

2. Install ROS 2 packages:
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop
    ```

3. Source the ROS 2 setup script:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
4. Install dependencies:
    ```bash
    sudo apt install python3-argcomplete python3-colcon-common-extensions
    ```

### Installation for Docker
> [!IMPORTANT]  
> Cross Check these installtion commands from the official Docker installation guide
#### 1. Install Docker
Docker is required to run the ROS 2 application in a containerized environment. Follow the official Docker [installation guide](https://docs.docker.com/desktop/install/linux-install/):

1. Update your package index:
    ```bash
    sudo apt-get update
    ```

2. Install required packages:
    ```bash
    sudo apt-get install \
        ca-certificates \
        curl \
        gnupg \
        lsb-release
    ```
 3. Add Docker’s official GPG key:

    ```bash
    sudo mkdir -p /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gp
    ```

 4. Set up the Docker repository:

    ```bash
    echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    ```

5. Install Docker Engine:
    ```bash
    sudo apt-get update
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```

6. Verify Docker installation:
    ```bash
    sudo docker run hello-world
    ```

#### 2. Set Up NVIDIA Docker (Optional)
> [!IMPORTANT]  
> Cross Check these installtion commands from the official NVIDIA Docker installation guide
If you're using GPU acceleration, you'll need to set up NVIDIA Docker. Follow the official Nvidia Container Toolkit [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):

1. Install NVIDIA container toolkit:
    ```bash
    sudo apt-get install -y nvidia-container-toolkit
    sudo systemctl restart docker
    ```

2. Test NVIDIA Docker setup:
    ```bash
    sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
    ```

#### 3. Clone the Repository
Clone this repository to your local machine:
```bash
git clone https://github.com/your-repo/universal-robotics-arm.git
cd universal-robotics-arm
```

#### 4. Running the container
To run the container there is a convenient python script:

> [!NOTE]
> Running the container first time will start building the image, this will take several minutes.

1. Starting the Container
    ```bash
    ./launch start
    ```

2. Enter the Container
    ```bash
    ./launch exec
    ```
### Installation for Windows and MacOS
> I Like your enthusiasm, Best of Luck for your future endeavors!

## Running the Package

### 1. Loading the Robot in RVIZ
To visualize the robot in Rviz run the following command:
```bash
ros2 launch ur_description robot_rviz.launch.py model:=<MODEL OF THE ROBOT>
```

### 2. Loading the Robot in Gazebo
To start simulating the robot in Gazebo run the following command:
```bash
ros2 launch ur_gz_bringup sil.launch.py model:=<MODEL OF THE ROBOT>
```
To start simulating the robot in Gazebo along with rviz run the following command:
```bash
ros2 launch ur_gz_bringup sil.launch.py model:=<MODEL OF THE ROBOT> rviz:=true
```
## License 
This project is licensed under the GPL-3.0-only License. See the [LICENSE](LICENSE) file for more details.

## Acknowledgements
This project is developed based on resources and support from Universal Robotics. We extend our gratitude to them for providing the robotic arm and necessary documentation for the development of this ROS package.
