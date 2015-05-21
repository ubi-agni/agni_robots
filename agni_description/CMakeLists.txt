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
file(GLOB model_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/sfb.urdf.xacro)

foreach(it ${model_xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)
  
  message(" processing model file :" , ${basepath} ," : " , ${basename})
  
  set(expanded_file "${basepath}/${basename}.urdf")
  message("    produce expanded file :" , ${expanded_file})
  
  xacro_add_xacro_file(${it} ${expanded_file})
  
  set(expanded_xacro_files ${expanded_xacro_files} ${expanded_file})
endforeach(it)

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
