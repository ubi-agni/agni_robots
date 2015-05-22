cmake_minimum_required(VERSION 2.8.3)
project(agni_description)
find_package(catkin REQUIRED COMPONENTS urdf xacro)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS urdf xacro
    INCLUDE_DIRS
    LIBRARIES
)

set(expanded_xacro_files "")

# Xacro files

# sfb 
#xacro_add_target("robots/sfb.urdf.xacro" "sfb.urdf" INORDER )
file(GLOB cur_file ${CMAKE_CURRENT_SOURCE_DIR}/robots/sfb.urdf.xacro)
message(" processing model file :" , ${cur_file})
get_filename_component(basepath ${cur_file} PATH)
set(expanded_file "${basepath}/generated/sfb.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER)
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})
# famula
file(GLOB cur_file ${CMAKE_CURRENT_SOURCE_DIR}/robots/famula.urdf.xacro)
message(" processing model file :" , ${cur_file})
get_filename_component(basepath ${cur_file} PATH)
set(expanded_file "${basepath}/generated/famula.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER)
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})


#sfbframe
file(GLOB cur_file ${CMAKE_CURRENT_SOURCE_DIR}/robots/frame.urdf.xacro)
message(" processing model file :" , ${cur_file})
get_filename_component(basepath ${cur_file} PATH)
set(expanded_file "${basepath}/generated/sfb_frame.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP FRAME:=pa10)
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})
#famulaframe
set(expanded_file "${basepath}/generated/famula_frame.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP FRAME:=kuka)
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

# left right pa10
file(GLOB cur_file ${CMAKE_CURRENT_SOURCE_DIR}/robots/arms.urdf.xacro)
message(" processing model file :" , ${cur_file})
get_filename_component(basepath ${cur_file} PATH)
set(expanded_file "${basepath}/generated/right_pa10.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP ARM:=pa10 SIM:=1 SIDE:=right)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

set(expanded_file "${basepath}/generated/left_pa10.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP ARM:=pa10 SIM:=1 SIDE:=left)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

# left right kuka
set(expanded_file "${basepath}/generated/right_kuka.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP ARM:=kuka SIM:=1 SIDE:=right)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

set(expanded_file "${basepath}/generated/left_kuka.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP ARM:=kuka SIM:=1 SIDE:=left)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})


#left right shadow_hand_motor_ubi
file(GLOB cur_file ${CMAKE_CURRENT_SOURCE_DIR}/robots/shadow_hands.urdf.xacro)
message(" processing model file :" , ${cur_file})
get_filename_component(basepath ${cur_file} PATH)
set(expanded_file "${basepath}/generated/right_shadow_hand_motor_ubi.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP MODEL:=motor SIDE:=right PREFIX:=rh TACTILE:=ubi)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

set(expanded_file "${basepath}/generated/left_shadow_hand_motor_ubi.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP MODEL:=motor SIDE:=left PREFIX:=lh TACTILE:=ubi )  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

#left right pa10 shadow_hand_motor_ubi
file(GLOB cur_file ${CMAKE_CURRENT_SOURCE_DIR}/robots/arms_hands.urdf.xacro)
message(" processing model file :" , ${cur_file})
get_filename_component(basepath ${cur_file} PATH)
set(expanded_file "${basepath}/generated/right_pa10_shadow_hand_motor_ubi.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP ARM:=pa10 SIM:=1 TACTILE:=ubi HANDMODEL:=motor SIDE:=right)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

set(expanded_file "${basepath}/generated/left_pa10_shadow_hand_motor_ubi.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP ARM:=pa10 SIM:=1 TACTILE:=ubi HANDMODEL:=motor SIDE:=left)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

#left right kuka shadow_hand_motor_ubi
set(expanded_file "${basepath}/generated/right_kuka_shadow_hand_motor_ubi.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP ARM:=kuka SIM:=1 TACTILE:=ubi HANDMODEL:=motor SIDE:=right)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})

set(expanded_file "${basepath}/generated/left_kuka_shadow_hand_motor_ubi.urdf")
message("    produce expanded file :" , ${expanded_file})
xacro_add_xacro_file(${cur_file} ${expanded_file} INORDER REMAP ARM:=kuka SIM:=1 TACTILE:=ubi HANDMODEL:=motor SIDE:=left)  
set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})



add_custom_target(media_files ALL DEPENDS ${expanded_xacro_files})


#catkin_add_gtest(test_sr_description_urdf test/test_sr_description_urdf.cpp)
#add_dependencies(test_sr_description_urdf ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_sr_description_urdf ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)

install(DIRECTORY doc/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc)

install(DIRECTORY kuka/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/kuka)

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
   

#file(GLOB def_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro
#                    ${CMAKE_CURRENT_SOURCE_DIR}/robots/*groups*.xacro)

#file(GLOB xacros ${CMAKE_CURRENT_SOURCE_DIR}/robots/*.urdf.xacro)

#rosbuild_find_ros_package(xacro)

#set(outfiles "")

#foreach( _xacro ${xacros} )
#  string(REPLACE ".urdf." ".expanded." _outfile ${_xacro})
#  add_custom_command(
#    OUTPUT ${_outfile}
#    COMMAND ${xacro_PACKAGE_PATH}/xacro.py
#    ARGS ${_xacro} > ${_outfile}
#    DEPENDS ${_xacro} ${def_files})
#  list(APPEND outfiles ${_outfile})
#endforeach( _xacro)

#add_custom_target( expanded_defs ALL DEPENDS ${outfiles} )
