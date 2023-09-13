cmake_minimum_required(VERSION 3.0.2)
project(adeept_noetic)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/ros_engine/move.py
  src/ros_engine/ultrasonic.py
  src/ros_engine/led_controller.py
  src/ros_engine/findline.py
  src/ros_engine/controller_nothread.py
  src/ros_engine/controller.py
  src/ros_engine/camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
