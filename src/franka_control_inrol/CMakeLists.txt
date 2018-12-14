cmake_minimum_required(VERSION 2.8.3)
project(franka_control_inrol)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  rosbag
  geometry_msgs
  genmsg
  diagnostic_updater
  std_msgs
)

add_service_files(FILES
    String_cmd.srv
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.3.0 REQUIRED)

add_message_files(FILES
  JointTorqueComparison.msg
  JointPkg.msg
  TorquePkg.msg
  Vector6.msg
)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_control_inrol
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    rospy
    rosbag
    geometry_msgs
    diagnostic_updater
    diagnostic_msgs
    message_runtime
    tf
  DEPENDS Franka
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")        ## Optimize

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/netft_rdt_driver/netft_rdt_driver.h
                 ${INCLUDE_DIR}/netft_rdt_driver/netft_rdt_bias.h
                 ${INCLUDE_DIR}/netft_rdt_driver/ft_listener.h
)

set(SRC_FILES   ${SRC_DIR}/netft_rdt_driver.cpp
                ${SRC_DIR}/netft_rdt_bias.cpp
                ${SRC_DIR}/ft_listener.cpp
)


add_library(franka_control_inrol
  #src/txt_out.cpp
  src/inrol_impedance_controller.cpp
  src/parameter_identification.cpp
  src/inrol_peg_in_hole.cpp
  src/path_generator.cpp
  src/impedance_controller_6DOF.cpp
  #src/peg_in_hole_planning.cpp
  src/cartesian_pose_controller.cpp
  ${SRC_FILES}
  ${HEADER_FILES}
)


add_dependencies(franka_control_inrol
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_control_inrol PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)


target_include_directories(franka_control_inrol SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(franka_control_inrol PUBLIC
  include
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  ${KALMAN_INCLUDE}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(peg_in_hole_planning src/peg_in_hole_planning.cpp)
target_link_libraries(peg_in_hole_planning ${catkin_LIBRARIES})

add_executable(txt_out src/txt_out.cpp)
target_link_libraries(txt_out ${catkin_LIBRARIES})

add_executable(netft_with_franka_node ${SRC_DIR}/netft_node.cpp)
target_link_libraries(netft_with_franka_node ${PROJECT_NAME} boost_program_options)





## Installation
install(TARGETS franka_control_inrol
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_control_inrol_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_control_inrol FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_control_inrol
    FILES ${SOURCES}
    DEPENDS franka_control_inrol
  )
endif()
