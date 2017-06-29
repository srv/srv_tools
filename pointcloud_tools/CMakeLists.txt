cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_tools)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros std_msgs sensor_msgs nav_msgs pcl_conversions)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

find_package(Boost REQUIRED COMPONENTS signals filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})

add_executable(pointcloud_mapper src/pointcloud_mapper.cpp)
add_executable(pointcloud_mapper_for_slam src/pointcloud_mapper_for_slam.cpp)
add_executable(pointcloud_filtering src/pointcloud_filtering.cpp)
add_executable(pcd_publisher src/pcd_publisher.cpp)
add_executable(pointcloud_viewer src/pointcloud_viewer.cpp)
add_executable(pointcloud_to_webgl src/pointcloud_to_webgl.cpp)

target_link_libraries(pointcloud_viewer ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(pcd_publisher ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pointcloud_mapper ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pointcloud_mapper_for_slam ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pointcloud_filtering ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pointcloud_to_webgl ${PCL_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS pointcloud_viewer pcd_publisher pointcloud_mapper pointcloud_mapper_for_slam pointcloud_filtering pointcloud_to_webgl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)