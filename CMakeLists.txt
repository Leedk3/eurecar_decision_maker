cmake_minimum_required(VERSION 2.8.12)
project(eurecar_decision_maker)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  amathutils_lib
  autoware_config_msgs
  autoware_msgs
  dynamic_reconfigure
  geometry_msgs
  jsk_recognition_msgs
  jsk_rviz_plugins
  lanelet2_extension
  roscpp
  roslint
  state_machine_lib
  std_msgs
  tf
  vector_map
  vector_map_msgs
)

catkin_package()

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp(test/src/test_node_lanelet2_functions.cpp)

include_directories(
  include/decision_maker
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  nodes/decision_maker/decision_maker_node.cpp
  nodes/decision_maker/decision_maker_node_core.cpp
  nodes/decision_maker/decision_maker_node_decision.cpp
  nodes/decision_maker/decision_maker_node_init.cpp
  nodes/decision_maker/decision_maker_node_callback.cpp
  nodes/decision_maker/decision_maker_node_state_vehicle.cpp
  nodes/decision_maker/decision_maker_node_publish.cpp
  nodes/decision_maker/cross_road_area.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  FILES
  cfg/eurecar_state_vehicle.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

