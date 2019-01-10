cmake_minimum_required(VERSION 2.8.3)

project(mara_bringup)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/follow_joints_individual_trajectory.py
  scripts/follow_joints_module.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
