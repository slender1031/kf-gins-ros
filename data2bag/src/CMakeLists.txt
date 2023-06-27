cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(data_convert)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")


find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(data_convert_node  data2bag.cpp)
add_dependencies(data_convert_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(data_convert_node
${catkin_LIBRARIES}
)



