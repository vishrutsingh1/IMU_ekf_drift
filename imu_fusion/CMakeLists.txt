cmake_minimum_required(VERSION 3.0.2)
project(imu_fusion)

find_package(catkin REQUIRED COMPONENTS rospy nav_msgs robot_localization)

catkin_package()

catkin_install_python(PROGRAMS src/imu_fusion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
