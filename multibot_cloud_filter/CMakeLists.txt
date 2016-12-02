cmake_minimum_required(VERSION 2.8.3)
project(multibot_cloud_filter)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  pcl_ros
  roscpp
  tf
)

find_package(PCL 1.7 REQUIRED REQUIRED)
add_definitions(${PCL_DEFINITIONS})


###################################
## catkin specific configuration ##
###################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(multibot_cloud_filter src/multibot_cloud_filter.cpp)
add_dependencies(multibot_cloud_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multibot_cloud_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS multibot_cloud_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
