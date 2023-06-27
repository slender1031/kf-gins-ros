cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(gins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3)
include_directories(
  gins
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_library(gins_lib
  gins/gins_engine.cpp
  gins/INSMechan.cpp
  gins/parameters.cpp
  gins/visualization.cpp
)
target_link_libraries(gins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} )


add_executable(gins_node  gins.cpp)
target_link_libraries(gins_node gins_lib) 



