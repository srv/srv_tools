cmake_minimum_required(VERSION 2.8.3)
project(tf_tools)

find_package(catkin REQUIRED COMPONENTS tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_logger src/tf_logger.cpp)

target_link_libraries(tf_logger ${catkin_LIBRARIES})

