cmake_minimum_required(VERSION 2.8.3)
project(cost_map_exercise)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  grid_map_msgs
  grid_map_ros
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    costmap_2d
    grid_map_msgs
    grid_map_ros
)

include_directories(
  include
  include/simple_layers
  ${catkin_INCLUDE_DIRS}
)

add_library(grid_layer src/simple_layers/grid_layer.cpp)

add_dependencies(grid_layer grid_map_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS grid_layer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
