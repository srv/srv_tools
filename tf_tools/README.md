Systems, Robotics and Vision Group (SRV)
========================================

Programs, libraries and utilities for ROS

tf_tools
--------

### Description

tf_logger will record the changes of two frames published in tf each with respect to its own reference and dump it as text into an output file.

### Usage

Run:

    $ rosrun extrinsic_calibration tf_logger

Parameters:

* ~/baseLinkFrame (string, default: "/base_link"): First reference frame to log.
* ~/odomFrame (string, default: "/odom"): Parent frame for the first reference frame.
* ~/kinectFrame (string, default: "/openni_rgb_optical_frame"): Second reference frame to log.
* ~/worldFrame (string, default: "/world"): Parent frame for the second reference frame.
* ~/outputFileName (string, default: "output.txt")
