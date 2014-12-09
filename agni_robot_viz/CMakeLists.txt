cmake_minimum_required(VERSION 2.8.3)
project(agni_robot_viz)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_msgs
  eigen_conversions
  tf
  roscpp
  std_msgs
  visualization_msgs
  
)

set(rst-converters_DIR /vol/famula/nightly/share/rst-converters0.11)

## System dependencies are found with CMake's conventions
find_package(RSB REQUIRED)
find_package(RSC REQUIRED)
find_package(RST REQUIRED COMPONENTS sandbox)
find_package(rst-converters REQUIRED)

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
  INCLUDE_DIRS include
#  LIBRARIES agni_robot_viz
  CATKIN_DEPENDS geometry_msgs moveit_core moveit_ros_planning moveit_msgs roscpp tf std_msgs visualization_msgs eigen_conversions
  DEPENDS RSB RSC RST
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS} ${RSB_INCLUDE_DIRS} ${RSC_INCLUDE_DIRS} ${RST_INCLUDE_DIRS} ${RSTSANDBOX_INCLUDE_DIRS} ${RST_CONVERTERS_INCLUDE_DIRS}
)

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS} ${RST_CONVERTERS_CFLAGS})

## Declare a cpp library
add_library(collision_shape_generator src/collision_shape_generator.cpp)
add_dependencies(collision_shape_generator ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_shape_generator ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(collision_shape_generator_ros src/collision_shape_generator_ros.cpp)
add_dependencies(collision_shape_generator_ros 
    agni_robot_viz_generate_messages_cpp
    collision_shape_generator)
target_link_libraries(collision_shape_generator_ros
  collision_shape_generator
   ${catkin_LIBRARIES} 
)

## Declare a cpp executable
add_executable(collision_shape_generator_rsb src/collision_shape_generator_rsb.cpp)
add_dependencies(collision_shape_generator_rsb 
    agni_robot_viz_generate_messages_cpp
    collision_shape_generator)
target_link_libraries(collision_shape_generator_rsb
  collision_shape_generator
   ${catkin_LIBRARIES} ${RSB_LIBRARIES} ${RSC_LIBRARIES} ${RST_LIBRARIES} ${RSTSANDBOX_LIBRARIES} ${RST_CONVERTERS_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS collision_shape_generator_ros collision_shape_generator_rsb collision_shape_generator
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


## Mark executables and/or libraries for installation
# install(TARGETS agni_robot_viz agni_robot_viz_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_agni_robot_viz.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
