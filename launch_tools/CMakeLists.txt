cmake_minimum_required(VERSION 3.0.2)
project(launch_tools)

find_package(catkin REQUIRED COMPONENTS rospy roslib)

catkin_package()

catkin_python_setup()

install(PROGRAMS
    scripts/launch_tools/launchViz.py
    scripts/launch_tools/services_timer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)