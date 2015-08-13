cmake_minimum_required(VERSION 2.8.3)
project(agni_rtt_services)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm rtt_ros rtt_controller_manager_msgs genmsg message_generation) 

add_message_files(
  FILES
  ControlIOType.msg
  ControlIOMap.msg
)

generate_messages()

find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include_directories(${catkin_INCLUDE_DIRS})

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

ros_generate_rtt_typekit(agni_rtt_services)

orocos_use_package(ocl-deployment)

orocos_plugin(rttlua_controller_service src/rttlua_controller_service.cpp)
add_dependencies(rttlua_controller_service agni_rtt_services_generate_messages_cpp)
target_link_libraries(rttlua_controller_service ${catkin_LIBRARIES} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})

orocos_plugin(rtt_controller_manager_service src/rtt_controller_manager.cpp)
add_dependencies(rtt_controller_manager_service controller_manager_msgs_generate_messages_cpp)
target_link_libraries(rtt_controller_manager_service ${catkin_LIBRARIES} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})


catkin_package(
  DEPENDS ocl rtt
  rtt_roscomm rtt_controller_manager_msgs
)

