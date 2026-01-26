cmake_minimum_required(VERSION 3.0.2)
project(bag_tools)


##############
## Compiler ##
##############

# C++17 Standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")


#######################
## Required packages ##
#######################

find_package(catkin REQUIRED COMPONENTS 
  rospy
  roscpp
  rosbag
  cv_bridge
  image_proc
  sensor_msgs
  image_geometry
  message_filters 
  stereo_image_proc 
  camera_calibration_parsers
  
  image_preprocessing # Optional
)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)

# Dependencies - Boost:
find_package(Boost REQUIRED COMPONENTS thread)

# Dependencies - console_bridge:
find_package(console_bridge REQUIRED)

# Dependencies - image_preprocessing:
# find_package(image_preprocessing REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosbag
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

add_executable(extract_images src/extract_images.cpp)
add_executable(extract_images_from_realsense src/extract_images_from_realsense.cpp)
add_executable(extract_tum_format src/extract_tum_format.cpp)
add_executable(extract_stereo_images
               src/image_resolution_scaler.cpp
               src/image_saver.cpp
               src/stereo_image_saver.cpp
               src/extract_stereo_images.cpp)
add_executable(process_stereo src/process_stereo.cpp)
add_executable(extract_enhanced_stereo_images
               src/image_resolution_scaler.cpp
               src/enhanced_image_saver.cpp
               src/enhanced_stereo_image_saver.cpp
               src/extract_enhanced_stereo_images.cpp) # Optional
               
add_dependencies(extract_enhanced_stereo_images ${catkin_EXPORTED_TARGETS})

target_link_libraries(extract_images ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
target_link_libraries(extract_images_from_realsense ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
target_link_libraries(extract_stereo_images ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
target_link_libraries(extract_tum_format ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
target_link_libraries(process_stereo ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
target_link_libraries(extract_enhanced_stereo_images ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES}) # Optional


install(TARGETS
    extract_images
    extract_images_from_realsense
    extract_tum_format
    extract_stereo_images
    process_stereo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
    scripts/add_header_time_offset.py
    scripts/bag_add_time_offset.py
    scripts/batch_process.py
    scripts/camera_info_parser.py
    scripts/change_camera_info_folder.py
    scripts/change_camera_info.py
    scripts/change_frame_id.py
    scripts/change_topics.py
    scripts/check_delay.py
    scripts/check_drop.py
    scripts/cut.py
    scripts/extract_topics.py
    scripts/gps_to_std_gt.py
    scripts/image_sequence_publisher.py
    scripts/make_video.py
    scripts/merge.py
    scripts/plot.py
    scripts/remove_tf.py
    scripts/replace_msg_time_with_hdr.py
    scripts/sort_topics_by_timestamp.py
    scripts/stereo_sequence_publisher.py
    scripts/transform_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)