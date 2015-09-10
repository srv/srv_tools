cmake_minimum_required(VERSION 2.8.3)
project(bag_tools)

find_package(catkin REQUIRED COMPONENTS rospy rosbag sensor_msgs cv_bridge message_filters image_proc stereo_image_proc image_geometry camera_calibration_parsers)
find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)
find_package(console_bridge REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosbag
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

add_executable(extract_images src/extract_images.cpp)
add_executable(extract_stereo_images src/extract_stereo_images.cpp)
add_executable(process_stereo src/process_stereo.cpp)

target_link_libraries(extract_images ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
target_link_libraries(extract_stereo_images ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
target_link_libraries(process_stereo ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})

install(TARGETS
    extract_images
    extract_stereo_images
    process_stereo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
    scripts/stereo_sequence_publisher.py
    scripts/replace_msg_time_with_hdr.py
    scripts/remove_tf.py
    scripts/make_video.py
    scripts/image_sequence_publisher.py
    scripts/gps_to_std_gt.py
    scripts/extract_topics.py
    scripts/cut.py
    scripts/check_delay.py
    scripts/change_topics.py
    scripts/change_frame_id.py
    scripts/change_camera_info.py
    scripts/camera_info_parser.py
    scripts/batch_process.py
    scripts/bag_add_time_offset.py
    scripts/add_header_time_offset.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

