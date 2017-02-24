^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bag_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2017-02-24)
------------------
* Fix release
* Add missing changes
* Forgot to remove extract_image_positions target
* Fix `#12 <https://github.com/srv/srv_tools/issues/12>`_: Remove old field from camera_info msg
* Remove extract_image_positions and auv_msgs depencency
* add auv_msgs deps in CMake
* Fix `#10 <https://github.com/srv/srv_tools/issues/10>`_
* add extract_image_positions
* Merge branch 'indigo' of github.com:srv/srv_tools into indigo
* Minnor changes
* Merge branch 'indigo' of github.com:srv/srv_tools into indigo
* add missing rospy node init
* Merge branch 'indigo' of github.com:srv/srv_tools into indigo
* Fix camera info tool
* Fix bag_tools scripts install
* Added stereo_sequence_publisher.py
* fix seq publisher for indigo
* Addapt the pointlcoud_to_webgl script to the new format of the website
* Contributors: Enrique Fernandez, Miquel Massot, matlabbe, plnegre

0.0.2 (2017-02-23)
------------------
* Add missing changes
* Forgot to remove extract_image_positions target
* Fix `#12 <https://github.com/srv/srv_tools/issues/12>`_: Remove old field from camera_info msg
* Remove extract_image_positions and auv_msgs depencency
* add auv_msgs deps in CMake
* Fix `#10 <https://github.com/srv/srv_tools/issues/10>`_
* add extract_image_positions
* Merge branch 'indigo' of github.com:srv/srv_tools into indigo
* Minnor changes
* Merge branch 'indigo' of github.com:srv/srv_tools into indigo
* add missing rospy node init
* Merge branch 'indigo' of github.com:srv/srv_tools into indigo
* Fix camera info tool
* Fix bag_tools scripts install
* Added stereo_sequence_publisher.py
* fix seq publisher for indigo
* Addapt the pointlcoud_to_webgl script to the new format of the website
* Contributors: Enrique Fernandez, Miquel Massot, matlabbe, plnegre

* typo on console_bridge dependency
* added -this- to reference the same class
* Of course, we must clear the tmp folder...
* This is the "intersting" patch I did : Now extract_image works also with rosbag containing compressed images, if these images are decodable by opencv (this is the case for JPG and PNG compression).
* In this python script, I added "import rospy" to be able to run this script in a Catkin environment.
  Then, I removed the option "sameq" of the ffmpeg process. In ubuntu 14.04, ffmpeg is now called avconv, on my computer I have a symlink from avconv to ffmpeg, so calling ffmpeg still work. But with this version of FFMPEG (avconv 9.16), there is nolonger the sameq argument. Maybe it could be interesting to call avconc and rather than ffmpeg is ffmpeg executable is not found...
* Syntax error fixed in this script.
* Changes for indigo
* preparing for indigo. changed prints for ros logs
* added gps_to_std_gt and services_timer from fuerte
* added missing dependency
* hydro catkinization changes
* added python setup files and wet'ed plot tools
* wet repo
* new script for topic extraction
* remove_tf now removes child tfs as well
* new script for changing a topic in a bagfile
* added bag processor for single cameras
* added export flags to manifest
* made camera info optional
* added batch processing script
* new script for tf manipulation
* improved help text
* added script for making videos from bagfiles
* removed unused method
* new binary for image extraction
* changed logging output
* added param for file pattern
* added link to boost signals
* added some more processing
* new executable for stereo processing inbag->outbag, some refactoring
* added license for cut script
* Merge branch 'master' of github.com:srv/srv_tools
* changed manifests
* added BSD license
* new script for cutting bagfiles
* removed unused import
* changed parameter handling for image sequence publisher node to use rosparams instead of command line arguments
* added parsers to the python scripts
* added main README and bag_tools README
* fixed boost dependency for fuerte
* removed extract_images script as now there is a better c++ code for that
* new binary for extraction of stereo images from a bagfile
* new image sequence publisher
* added camera info parser as seperate script
* fixed launch, added cv_bridge to manifest
* added launch files for image extraction
* fixed wrong package name
* new script for header time modification
* added median calc and support for /tf topic
* added support for multiple bagfile input in check_delay
* new script for delay check
* new script for camera info changing
* bugfix, output bagfile contained only changed topics before
* bugfixes
* added script: change frame id of topics in bagfile
* added script: replaces bagfile message time with header timestamp
* first working version of bag_add_time_offset.py
* added missing packages in manifest
* initial commit
* Contributors: Miquel Massot, Stephan Wirth, aba
