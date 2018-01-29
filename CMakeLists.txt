cmake_minimum_required(VERSION 2.8.3)
project(cost_map_exercise)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  grid_map_msgs
  grid_map_ros
  grid_map_core
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    costmap_2d
    grid_map_msgs
    grid_map_ros
    grid_map_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cost_map_exercise src/grid_layer.cpp include/simple_layers/grid_layer.hpp)

add_dependencies(cost_map_exercise grid_map_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS cost_map_exercise
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
