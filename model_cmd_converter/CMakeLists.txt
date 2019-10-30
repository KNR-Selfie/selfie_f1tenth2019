cmake_minimum_required(VERSION 2.8.3)
project(model_cmd_converter)

find_package(catkin REQUIRED
  std_msgs
  roscpp
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(model_cmd_converter src/model_cmd_converter.cpp)
target_link_libraries(model_cmd_converter ${catkin_LIBRARIES})
