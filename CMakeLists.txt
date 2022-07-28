cmake_minimum_required(VERSION 3.0.2)
project(kinova_ar_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  kortex_driver
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES kinova_ar_teleop
   CATKIN_DEPENDS roscpp rospy std_msgs kortex_driver geometry_msgs tf2 tf2_ros
)

catkin_install_python(PROGRAMS scripts/hand_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  /usr/include/eigen3/
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/hololens_utility.cpp
)

add_executable(teleop_w_twist src/teleop_w_twist.cpp)
add_executable(test_pub src/test_pub.cpp)

target_link_libraries(
  teleop_w_twist 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}  
)

target_link_libraries(
  test_pub
  ${catkin_LIBRARIES}
)
