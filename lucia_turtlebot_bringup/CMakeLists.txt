cmake_minimum_required(VERSION 2.8.3)
project(lucia_turtlebot_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY camera_info
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
