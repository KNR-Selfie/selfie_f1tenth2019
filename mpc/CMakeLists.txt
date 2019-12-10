cmake_minimum_required(VERSION 2.8.3)
project(mpc)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  nav_msgs
  roscpp
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++11")


catkin_package(

)


include_directories( ${catkin_INCLUDE_DIRS} include )

add_executable(${PROJECT_NAME} src/mpc_node.cpp src/mpc.cpp )
target_link_libraries(${PROJECT_NAME} ipopt ${catkin_LIBRARIES})
