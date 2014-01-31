cmake_minimum_required(VERSION 2.8.3)
project(bag_tools)

find_package(catkin REQUIRED COMPONENTS rospy rosbag sensor_msgs cv_bridge message_filters image_proc stereo_image_proc image_geometry camera_calibration_parsers console_bridge)
find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosbag
)

catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(extract_images src/extract_images.cpp)
add_executable(extract_stereo_images src/extract_stereo_images.cpp)
add_executable(process_stereo src/process_stereo.cpp)

target_link_libraries(extract_images ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(extract_stereo_images ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(process_stereo ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

