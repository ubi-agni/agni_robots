cmake_minimum_required(VERSION 2.8.3)
project(agni_description)
find_package(catkin REQUIRED)
find_package(xacro 1.10 REQUIRED)
find_package(sr_description QUIET)
find_package(lwr_description QUIET)
find_package(pa10_7a_description QUIET)
find_package(pa10_7c_description QUIET)
find_package(schunk_description QUIET)
find_package(myrmex_description QUIET)
find_package(flobi_description QUIET)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package()

macro(xacro file)
   cmake_parse_arguments("X" "" "" "DEPENDS" ${ARGN})
   set(_missing "")
   foreach(dep ${X_DEPENDS})
      if(NOT ${dep}_FOUND)
         set(_missing "${_missing}${dep} ")
      endif()
   endforeach()
   if("${_missing}" STREQUAL "")
      xacro_add_xacro_file(${file} ${X_UNPARSED_ARGUMENTS})
      list(APPEND xacro_outputs ${XACRO_OUTPUT_FILE})
   else()
      message("Skipping generation of ${file} due to missing: ${_missing}")
   endif()
endmacro()

xacro(robots/sfb.urdf.xacro DEPENDS pa10_7a_description pa10_7c_description sr_description)
xacro(robots/famula.urdf.xacro DEPENDS lwr_description flobi_description)
xacro(robots/moveable_famula.urdf.xacro DEPENDS lwr_description flobi_description)

# sfb frame
xacro(robots/single.urdf.xacro robots/sfb_frame.urdf REMAP FRAME:=sfb)

# famula frame
xacro(robots/single.urdf.xacro robots/famula_frame.urdf REMAP FRAME:=famula)
xacro(robots/single.urdf.xacro robots/moveable_famula_frame.urdf REMAP FRAME:=moveable_famula)

# left right pa10
xacro(robots/single.urdf.xacro robots/right_pa10.urdf REMAP CALIB:=sfb ARM:=pa10c PREFIX:=r SIM:=1 DEPENDS pa10_7c_description)
xacro(robots/single.urdf.xacro robots/left_pa10.urdf REMAP CALIB:=sfb ARM:=pa10a PREFIX:=l SIM:=1 DEPENDS pa10_7a_description)
xacro(robots/bimanual.urdf.xacro robots/pa10.urdf REMAP  FRAME:=sfb ARM_R:=pa10c ARM_L:=pa10a FLOBI:=0 TOOL_R:=- TOOL_L:=- SIM:=1 DEPENDS pa10_7c_description pa10_7c_description)
# left right kuka
xacro(robots/single.urdf.xacro robots/right_kuka.urdf REMAP CALIB:=famula ARM:=kukaR PREFIX:=r SIM:=1 DEPENDS lwr_description)
xacro(robots/single.urdf.xacro robots/left_kuka.urdf REMAP CALIB:=famula ARM:=kukaL PREFIX:=l SIM:=1 DEPENDS lwr_description)
xacro(robots/bimanual.urdf.xacro robots/kuka.urdf REMAP  FRAME:=famula ARM_R:=kukaR ARM_L:=kukaL FLOBI:=0 TOOL_R:=- TOOL_L:=- SIM:=1 DEPENDS lwr_description)

# left right lwr (no auto_on)
xacro(robots/single.urdf.xacro robots/right_lwr.urdf REMAP CALIB:=famula ARM:=lwrR PREFIX:=r SIM:=1 DEPENDS lwr_description)
xacro(robots/single.urdf.xacro robots/left_lwr.urdf REMAP CALIB:=famula ARM:=lwrL PREFIX:=l SIM:=1 DEPENDS lwr_description)

# left right shadow motor hand
xacro(robots/single.urdf.xacro robots/right_shadow_hand_motor_ubi.urdf REMAP TOOL:=shadow_motor_right PREFIX:=r PARENT:=- DEPENDS sr_description)
xacro(robots/single.urdf.xacro robots/left_shadow_hand_motor_ubi.urdf REMAP TOOL:=shadow_motor_left PREFIX:=l PARENT:=- DEPENDS sr_description)

# left right pa10 shadow_hand_motor_ubi
xacro(robots/single.urdf.xacro robots/right_pa10_shadow_hand_motor_ubi.urdf REMAP CALIB:=sfb ARM:=pa10c TOOL:=shadow_motor_right PREFIX:=r SIM:=1 DEPENDS pa10_7c_description sr_description)
xacro(robots/single.urdf.xacro robots/left_pa10_shadow_hand_motor_ubi.urdf REMAP CALIB:=sfb ARM:=pa10a TOOL:=shadow_motor_left PREFIX:=l SIM:=1 DEPENDS pa10_7a_description sr_description)

# left right kuka shadow_hand_motor_ubi
xacro(robots/single.urdf.xacro robots/right_kuka_shadow_hand_motor_ubi.urdf REMAP CALIB:=famula ARM:=kukaR TOOL:=shadow_motor_right PREFIX:=r SIM:=1 DEPENDS lwr_description sr_description)
xacro(robots/single.urdf.xacro robots/left_kuka_shadow_hand_motor_ubi.urdf REMAP CALIB:=famula ARM:=kukaL TOOL:=shadow_motor_left PREFIX:=l SIM:=1 DEPENDS lwr_description sr_description)

# left right kuka shadow_hand_motor_ubi moveable platform
xacro(robots/single.urdf.xacro robots/moveable_right_lwr_shadow_hand_motor_ubi.urdf REMAP CALIB:=moveable_famula ARM:=lwrR TOOL:=shadow_motor_right PREFIX:=r SIM:=1 DEPENDS lwr_description sr_description)
xacro(robots/single.urdf.xacro robots/moveable_left_lwr_shadow_hand_motor_ubi.urdf REMAP CALIB:=moveable_famula ARM:=lwrL TOOL:=shadow_motor_left PREFIX:=l SIM:=1 DEPENDS lwr_description sr_description)


# install the generated files
xacro_install(xacro ${xacro_outputs} DESTINATION robots)


install(DIRECTORY doc/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc)

install(DIRECTORY kuka/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/kuka)

install(DIRECTORY shadow/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/shadow)

install(DIRECTORY schunk/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/schunk)

install(DIRECTORY pa10/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pa10)

install(DIRECTORY urdf/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(DIRECTORY models/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY calibration/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/calibration)

install(DIRECTORY robots/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# testing
enable_testing()

set(arms -)
if(pa10_7a_description)
  list(APPEND arms pa10a)
endif()
if(pa10_7c_description)
  list(APPEND arms pa10c)
endif()
if(lwr_description)
  list(APPEND arms kukaL kukaR)
endif()

set(tools -)
if(sr_description)
  list(APPEND tools shadow_motor_left shadow_motor_right shadow_muscle_left shadow_muscle_right poking_stick_ubi poking_stick_rigid)
endif()
if(schunk_description)
  list(APPEND tools schunk_hand_right schunk_hand_left)
endif()
if(myrmex_description)
  list(APPEND tools myrmex)
endif()

foreach(arm ${arms})
  foreach(tool ${tools})
    add_test(
      NAME test-${arm}-${tool}
      COMMAND ${CATKIN_ENV} ${_xacro_py} -o /dev/null ${CMAKE_CURRENT_SOURCE_DIR}/robots/single.urdf.xacro ARM:=${arm} TOOL:=${tool})
  endforeach()
endforeach()
