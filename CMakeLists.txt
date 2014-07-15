cmake_minimum_required(VERSION 2.8.3)
project(aruco_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  nodelet
  pal_vision_segmentation
  roscpp
  tf
)

find_package(aruco REQUIRED)
### Add a IF statement to ensure aruco version is more than or equal to 1.2.5 for compatibility ###

generate_dynamic_reconfigure_options(
  cfg/ArucoThreshold.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aruco
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure geometry_msgs image_transport pal_vision_segmentation roscpp tf
#  DEPENDS system_lib
)

include_directories(
  include
	${aruco_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#message(" aruco include: ${aruco_INCLUDE_DIRS} >>> LIBS: ${aruco_LIBS}")

## Declare a cpp library
add_executable(single src/simple_single.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(single ${PROJECT_NAME}_gencfg)
target_link_libraries(single aruco ${catkin_LIBRARIES})

add_executable(double src/simple_double.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(double ${PROJECT_NAME}_gencfg)
target_link_libraries(double aruco ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/nodelets/simple_board.cpp src/aruco_ros_utils.cpp)
target_link_libraries(${PROJECT_NAME} aruco ${catkin_LIBRARIES})
