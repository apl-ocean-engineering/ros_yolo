cmake_minimum_required(VERSION 2.8.3)
project(ros_yolo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  image_detection_msgs
)

 add_message_files(
  FILES
  prac.msg
)

generate_messages(
  DEPENDENCIES 
  sensor_msgs 
  std_msgs 
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_yolo
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs message_runtime image_detection_msgs
#  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/yolo.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

