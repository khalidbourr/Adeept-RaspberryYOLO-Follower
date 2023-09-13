# Human Following Robot using YOLOv5

This repository contains an implementation of a human following algorithm, allowing a robot to accurately track and follow a person. The algorithm leverages the power of the YOLOv5 object detection model.

## Robot

The primary robot used in this implementation is the Adept AWR mobile robot.

##Requirements

- **Operating System**: Raspbian or Ubuntu 20.04
- **Middleware**: ROS Noetic

##Features

- **Real-time Human Detection**: Using the YOLOv5 model to accurately detect humans in the robot's field of view.
- **Smooth Tracking**: The robot is capable of smoothly following a person without abrupt movements.
- **Safety Features**: The robot can navigate without colliding into obstacles while following the detected person.

## Setup & Installation

1. **Setup ROS Noetic**:
    - For Ubuntu 20.04: [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
    - For Raspbian: [ROS Noetic Installation Guide for Raspbian](URL_TO_GUIDE) *(Replace with actual link if there's a specific guide for Raspbian)*

2. **Clone the Repository**:
    ```bash
    git clone [your-repo-link]
    cd human_following
