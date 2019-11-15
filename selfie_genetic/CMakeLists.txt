cmake_minimum_required(VERSION 2.8.3)
project(selfie_genetic)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(catkin REQUIRED COMPONENTS
dynamic_reconfigure
nav_msgs
roscpp
std_msgs
tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CUDA_NVCC_FLAGS "" CACHE STRING "nvcc flags" FORCE)
SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC)

CUDA_INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
CUDA_ADD_EXECUTABLE(genetic src/kernel.cu)
target_link_libraries(genetic ${catkin_LIBRARIES})
# CUDA_ADD_EXECUTABLE(genetic_node src/genetic_node.cpp)
# CUDA_ADD_LIBRARY(helper src/helper_cuda.h)
# CUDA_ADD_LIBRARY(gputimer src/gputimer.h)
# CUDA_ADD_LIBRARY(helper_cuda  /usr/local/cuda-10.1/samples/common/inc/helper_cuda.h)
# CUDA_INCLUDE_DIRECTORIES(/usr/local/cuda-10.1/samples/common/inc)
