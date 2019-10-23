cmake_minimum_required(VERSION 2.8.3)
project(ghost-bridge)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED)
find_package(Guile REQUIRED)
find_package(AtomSpace REQUIRED)
find_package(CogServer REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ghost-bridge
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  include/ghost-bridge
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${GUILE_INCLUDE_DIRS}
)

add_library(Ghost
  lib/Ghost.cc
)

add_executable(${PROJECT_NAME}_node src/ghost-bridge_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  Ghost
  ${catkin_LIBRARIES}
  ${ATOMSPACE_LIBRARIES}
  ${COGSERVER_LIBRARIES}
  ${GUILE_LIBRARIES}
)

