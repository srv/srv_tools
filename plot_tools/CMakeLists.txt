cmake_minimum_required(VERSION 2.8.3)
project(plot_tools)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_python_setup()

install(PROGRAMS
    scripts/plot_tools/real_time_odometry_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)