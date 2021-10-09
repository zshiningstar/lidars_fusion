cmake_minimum_required(VERSION 3.0.2)
project(doubleLidars_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_filters
  sensor_msgs
  pcl_ros
  pcl_conversions
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES doubleLidars_fusion
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
#  /usr/include/pcl-1.7/pcl
)

add_executable(${PROJECT_NAME}_node src/double_lidars_fusion.cpp)
                                    
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
