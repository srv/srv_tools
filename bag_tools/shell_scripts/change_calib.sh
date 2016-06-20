#!/bin/bash

# Set your camera calibration files
L_CALIB="/data/HD3/bagfiles/turbot/2016_06_13_sant_feliu/l_calibration_water_bags.yaml"
R_CALIB="/data/HD3/bagfiles/turbot/2016_06_13_sant_feliu/r_calibration_water_bags.yaml"

# Set the directories where the bagfiles are
DIRS="13 14 15 16 17"



for d in $DIRS
do
  echo "---- DIRECTORY $d ----"

  mkdir corrected
  FILES=*.bag
  for f in $FILES
  do
    echo "Changing bag: $f"
    rosrun bag_tools change_camera_info.py $f corrected/$f /stereo_down/left/camera_info=$L_CALIB /stereo_down/right/camera_info=$R_CALIB
  done

done