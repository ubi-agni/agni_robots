cmake_minimum_required(VERSION 2.8.3)
project(agni_rtt_services)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_roscomm rtt_ros message_generation) 

include_directories(${catkin_INCLUDE_DIRS})

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

catkin_package(
  DEPENDS ocl rtt
  rtt_roscomm
)


include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )


orocos_use_package(ocl-deployment)

orocos_plugin(rttlua_controller_service src/rttlua_controller_service.cpp)
target_link_libraries(rttlua_controller_service ${catkin_LIBRARIES} ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})

add_dependencies(rttlua_controller_service rttlua_controller_service_generate_messages_cpp)

ros_generate_rtt_typekit(agni_rtt_services)
