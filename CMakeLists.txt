cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_cal)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  livox_ros_driver
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
  dynamic_reconfigure
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(aruco REQUIRED)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  config/TransformFilter.cfg  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lidar_camera_cal_lib
 CATKIN_DEPENDS geometry_msgs livox_ros_driver  nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf dynamic_reconfigure
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

find_library(APRILTAG_LIB apriltag)
if(NOT APRILTAG_LIB)
    message(FATAL_ERROR "AprilTag library not found")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ./include/lidar_camera_cal/
  /usr/local/include/
)


# ------------------------------------- tools -------------------------------------------

add_library(conversion_bridge_lib
  include include/tools/conversion_bridge.h
  lib/tools/conversion_bridge.cpp
)
add_library(rviz_draw_lib
  include include/tools/rviz_draw.h
  lib/tools/rviz_draw.cpp
)

add_dependencies(conversion_bridge_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rviz_draw_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(conversion_bridge_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(rviz_draw_lib
  ${catkin_LIBRARIES}
)

# ------------------------------------- lidar_camera_cal -------------------------------------------


# add_library(calibration_tool_lib
#   include include/calibration_tool.h
#   lib/calibration_tool.cpp
# )


# add_dependencies(calibration_tool_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# target_link_libraries(calibration_tool_lib
#   ${catkin_LIBRARIES}
# )



# ---------------------------------------- pointcloud_opr -----------------------------------------------

# add_library(point_cloud_process_lib
#   include include/pointcloud2_opr/point_cloud_process.h
#   # lib/pointcloud2_opr/point_cloud_process.tpp
# )


# add_dependencies(point_cloud_process_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# target_link_libraries(point_cloud_process_lib
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )


# ---------------------------------------- image_opr -----------------------------------------------

add_library(aruco_manager_lib
  include include/image_opr/aruco_manager.h
  lib/image_opr/aruco_manager.cpp
)

add_library(image_draw_lib
  include include/image_opr/image_draw.h
  lib/image_opr/image_draw.cpp
)

add_library(image_process_lib
  include include/image_opr/image_process.h
  lib/image_opr/image_process.cpp
)




add_dependencies(aruco_manager_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(image_draw_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(image_process_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(aruco_manager_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(image_draw_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(image_process_lib
  ${catkin_LIBRARIES}
)






## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide



# add_executable(pointcloud_process_node src/pointcloud_process_node.cpp)



# target_link_libraries(pointcloud_process_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   # calibration_tool_lib
#   corners_subscriber_publisher_lib
#   point_cloud_subscriber_publisher_lib
#   # point_cloud_process_lib
#   dynamic_reconfigure_lib
#   rviz_draw_3d_lib
#   recorder_lib
# )




add_executable(test1 src/test1.cpp)
# add_executable(calibration_node src/calibration_node.cpp)
add_executable(image_process_node src/image_process_node.cpp)
add_executable(pointcloud_process_node src/pointcloud_process_node.cpp)


target_link_libraries(test1
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  
)
# target_link_libraries(calibration_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   conversion_bridge_lib
#   calibration_tool_lib
#   aruco_manager_lib
#   image_draw_lib
#   image_process_lib
# )
target_link_libraries(image_process_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  conversion_bridge_lib
  aruco_manager_lib
  image_draw_lib
  rviz_draw_lib
  image_process_lib
)
target_link_libraries(pointcloud_process_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  conversion_bridge_lib
  rviz_draw_lib
)

# target_link_libraries(point_cloud_filter
#   ${catkin_LIBRARIES}
#   point_cloud_subscriber_publisher_lib
#   # point_cloud_process_lib
#   dynamic_reconfigure_lib
#   recorder_lib
# )
# target_link_libraries(rosbag_record_example
#   ${catkin_LIBRARIES}
#   point_cloud_subscriber_publisher_lib
#   # point_cloud_process_lib
#   dynamic_reconfigure_lib
#   recorder_lib
# )
# target_link_libraries(pcd_saver_example
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   point_cloud_subscriber_publisher_lib
#   # point_cloud_process_lib
#   dynamic_reconfigure_lib
#   recorder_lib
#   pcd_saver_lib
# )


# target_link_libraries(aruco_detection_example
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   draw_tool_lib
#   aruco_tool_lib
# )

# target_link_libraries(pointcloud_meshing
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/livox_pointcloud2_opr.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
