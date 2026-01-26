/// Copyright (c) 2012,
/// Systems, Robotics and Vision Group
/// University of the Balearican Islands
/// All rights reserved.
/// 
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * Neither the name of Systems, Robotics and Vision Group, University of 
///       the Balearican Islands nor the names of its contributors may be used 
///       to endorse or promote products derived from this software without 
///       specific prior written permission.
/// 
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_proc/processor.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_calibration_parsers/parse.h>

#include "bag_tools/stereo_image_saver.h"
#include "bag_tools/stereo_bag_processor.h"


int main(int argc, char** argv)
{
  if (argc < 8)
  {
    std::cout << "Saves rectified color images from raw and camera info input. It is possible to decimate images if the `decimation` parameter is greater than one." << std::endl;
    std::cout << "Usage: " << argv[0] << " OUT_DIR FILETYPE STEREO_BASE_TOPIC LEFT_CAMERA_NAME RIGHT_CAMERA_NAME DECIMATION BAGFILE [BAGFILE...]" << std::endl;
    std::cout << "  Example: " << argv[0] << " /tmp jpg /stereo_down /left /right 1 bag1.bag bag2.bag" << std::endl;
    return 0;
  }

  // Parsing.
  char *p;
  std::string out_dir(argv[1]);
  std::string filetype(argv[2]);
  std::string base_topic(argv[3]);
  std::string l_cam_name(argv[4]);
  std::string r_cam_name(argv[5]);
  long decimation_long = std::strtol(argv[6], &p, 10);
  int decimation = static_cast<int>(decimation_long);
  
  ros::Time::init();

  StereoImageSaver saver(out_dir, filetype, l_cam_name, r_cam_name, decimation);
  bag_tools::StereoBagProcessor processor(base_topic, l_cam_name, r_cam_name);
  processor.registerCallback(boost::bind(&StereoImageSaver::save, &saver, _1, _2, _3, _4));

  for (int i = 7; i < argc; ++i)
    processor.processBag(argv[i]);

  return 0;
}