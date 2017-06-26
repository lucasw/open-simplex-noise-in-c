cmake_minimum_required(VERSION 2.8.3)
project(open_simplex_noise_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
)

roslint_cpp(src/open_simplex_noise.cpp src/open_simplex_noise_node.cpp)

generate_dynamic_reconfigure_options(
  cfg/OpenSimplexNoise.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES open_simplex_noise_ros
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(open_simplex_noise_ros
  src/open_simplex_noise.cpp
  src/open-simplex-noise.c
)


add_dependencies(open_simplex_noise_ros ${PROJECT_NAME}_gencfg)

target_link_libraries(open_simplex_noise_ros
  ${catkin_LIBRARIES}
)

add_executable(open_simplex_noise_node src/open_simplex_noise_node.cpp)
target_link_libraries(open_simplex_noise_node
  ${catkin_LIBRARIES}
)

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(open_simplex_noise_ros ${catkin_EXPORTED_LIBRARIES})
endif()

# TODO(lucasw) install
