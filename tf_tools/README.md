tf_tools (SRV)
========

 Description
------------

tf_logger will record the changes of two frames published in tf each with respect to its own reference and dump it as text into an output file.

Usage
-----

* Run:

    $ rosrun extrinsic_calibration tf_logger

* Parameters:

1. ~/baseLinkFrame (string, default: "/base_link"): First reference frame to log.
2. ~/odomFrame (string, default: "/odom"): Parent frame for the first reference frame.
3. ~/kinectFrame (string, default: "/openni_rgb_optical_frame"): Second reference frame to log.
4. ~/worldFrame (string, default: "/world"): Parent frame for the second reference frame.
5. ~/outputFileName (string, default: "output.txt")
