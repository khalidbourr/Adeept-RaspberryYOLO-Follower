# Human Following Robot using YOLOv5

This repository contains an implementation of a human following algorithm, allowing a robot to accurately track and follow a person. The algorithm leverages the power of the YOLOv5 object detection model.

## Robot

The primary robot used in this implementation is the Adept AWR mobile robot.

##Requirements

- **Operating System**: Raspbian or Ubuntu 20.04
- **Middleware**: ROS Noetic
- **Python version**: Python 3.9

##Features

- **Real-time Human Detection**: Using the YOLOv5 model to accurately detect humans in the robot's field of view.
- **Smooth Tracking**: The robot is capable of smoothly following a person without abrupt movements.

## Setup & Installation

1. **Setup ROS Noetic**:
    - For Ubuntu 20.04: [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
    - For Raspbian: [ROS Noetic Installation Guide for Raspbian](See below)

2. **Clone the Repository**:
    ```bash
    git clone https://github.com/khalidbourr/Human-Following
    cd human_following
3. **Install Yolo requirements**:
    ```bash
    cd Human-Following/src/yolov5_ros/src/yolov5/
    pip3 install -r requirements.txt
3. **Prepare ROS Workspace:**:
    ```bash
    cd Human-Following
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
3. **Build Workspace:**:
    ```bash
    catkin build


## Install ROS Noetic on Raspberry Pi (Raspbian 10 - Buster)

```bash
#Setup ROS Repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#Install Dependencies
sudo apt update
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

# Download Dependencies
sudo rosdep init && rosdep update
mkdir ~/ros_catkin_ws && cd ~/ros_catkin_ws
rosinstall_generator ros_comm --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall
wstool init src noetic-ros_comm-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster

# Compile ROS
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3

# Add message packages
cd ~/ros_catkin_ws
rosinstall_generator navigation --rosdistro noetic --deps --wet-only --tar > noetic-navigation-wet.rosinstall
wstool merge noetic-navigation-wet.rosinstall -t src
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
catkin_make

# Note: If errors arise, consider commenting out problematic lines, typically in CMake files of the Geometry package.

# Add ROS environment variables to your .bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
