cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugin)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/simple_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(GAZEBOINCLUDEDIRS /usr/include;/usr/include/gazebo-7;/usr/include/sdformat-2.2)
set(GAZEBOLIBRARYDIRS /usr/lib/x8664-linux-gnu;/usr/lib/x8664-linux-gnu/gazebo-7/plugins)



