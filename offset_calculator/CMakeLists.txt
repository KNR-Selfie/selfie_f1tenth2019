cmake_minimum_required(VERSION 2.8.3)
project(offset_calculator)

find_package(
  catkin REQUIRED COMPONENTS
  tf
  nav_msgs
  std_msgs
  roscpp
  )

catkin_package(

)


include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(offset_calculator src/offset_calculator.cpp)
target_link_libraries(offset_calculator ${catkin_LIBRARIES})

add_executable(test_path_publisher src/test_path_publisher.cpp)
target_link_libraries(test_path_publisher ${catkin_LIBRARIES})

add_executable(test_transform_broadcast src/test_transform_broadcast.cpp)
target_link_libraries(test_transform_broadcast ${catkin_LIBRARIES})
