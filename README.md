# Human Following Robot using YOLOv5

This repository contains an implementation of a human following algorithm, allowing a robot to accurately track and follow a person. The algorithm leverages the power of the YOLOv5 object detection model.

## Robot

The primary robot used in this implementation is [Adeept AWR 4WD WiFi Smart Robot Car Kit for Raspberry Pi 4](https://www.adeept.com/adeept-awr-4wd-wifi-smart-robot-car-kit-for-raspberry-pi-3-model-b-b-2b-diy-robot-kit-for-kids-and-adults-opencv-target-tracking_p0122_s0033.html) ![Adeept AWR 4WD WiFi Smart Robot Car](https://uk.robotshop.com/cdn/shop/products/adeept-awr-4wd-wifi-smart-robot-car-kit-raspberry-pi_57849cb4-a8fa-4227-8406-9d46ef11930c_600x.jpg?v=1691709833)


## Requirements

- **Operating System**: Raspbian or Ubuntu 20.04, tested on 32 
- **Middleware**: ROS Noetic
- **Python version**: Python 3.9
- **Torch and open CV**: Compatible with ARM architecture


## Features

- **Real-time Human Detection**: Using the YOLOv5 model to accurately detect humans in the robot's field of view.
- **Smooth Tracking**: The robot is capable of smoothly following a person without abrupt movements.

## Setup & Installation

1. **Setup ROS Noetic**:
    - For Ubuntu 20.04: [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
    - For Raspbian: [ROS Noetic Installation Guide for Raspbian](See below)

2. **Clone the Repository**:
    ```bash
    git clone https://github.com/khalidbourr/Human-Following
    cd Human-Following
    
3. **Install OpenCV, Torch and Torchvision**:
   ```bash
    cd Human-Following
    pip3.9 install opencv_python-4.5.1.48-cp39-cp39-linux_armv7l.whl
    pip3.9 install torch-1.8.1-cp39-cp39-linux_armv7l.whl
    pip3.9 install torchvision-0.9.1-cp39-cp39-linux_armv7l.whl
    - 
4. **Install Yolo requirements**:
    ```bash
    cd Human-Following/src/yolov5_ros/src/yolov5/
    pip3 install -r requirements.txt
5. **Prepare ROS Workspace:**:
    ```bash
    cd Human-Following
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
6. **Build Workspace:**:
    ```bash
    catkin build


## Distributed Architecture Overview

To optimize performance, especially when it comes to processing-intensive tasks like object detection using YOLOv5, this project utilizes a distributed architecture. This approach divides the computational responsibilities between the Raspberry Pi and an external laptop. Here's why and how it's set up:

### Why Use a Distributed Architecture?

- **Performance Constraints of Raspberry Pi**: The Raspberry Pi, even in its latest models like the one we're using, is not equipped with a dedicated GPU. This limitation makes the object detection task using neural networks like YOLO quite slow and potentially impractical for real-time applications on the robot.
  
- **Leveraging External Computing Power**: By offloading the heavy computational task of object detection to a more powerful laptop equipped with a GPU, we achieve faster detection times. This way, the robot can react in real-time while following a human.

### How It Works:

1. **Prepare Three Terminals with Raspberry Pi Connection**:
    In each terminal:
    ```bash
    ssh -X Pi@[IP_of_your_robot]
    export ROS_MASTER_URI=http://[robot_IP]:11311
    export ROS_IP=[robot_IP]
    
2. **Prepare the Fourth Terminal (Without Raspberry Connection)**:
    ```bash
    export ROS_MASTER_URI=http://[robot_IP]:11311
    export ROS_IP=[laptop_IP]
#### Execution:
 
1. **Start the ROS Master (First Terminal)**:
    ```bash
    roscore
2. **Launch the Robot's Essential Components (Second Terminal)**:
    ```bash
    cd Human-Following/
    source devel/setup.bash
    roslaunch adeept_noetic system.launch
    
3. **Run YOLOv5 on the External Laptop (Fourth Terminal)**:
    ```bash
    cd [path_to_yolo_installation]
    source devel/setup.bash
    roslaunch yolov5_ros yolov5.launch input_image_topic:=/img
4. **Execute the Human Following Script (Third Terminal)**:
    ```bash
    cd Human-Following/
    source devel/setup.bash
    rosrun object_follower object_follower.py



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
