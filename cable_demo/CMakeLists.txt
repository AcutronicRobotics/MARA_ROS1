cmake_minimum_required(VERSION 2.8.3)
project(cable_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cable_demo
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/IK_moveit.cpp
)

add_executable(IK_moveit src/IK_moveit.cpp)
target_link_libraries(IK_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS IK_moveit DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
