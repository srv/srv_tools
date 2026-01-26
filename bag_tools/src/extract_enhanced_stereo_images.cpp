#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_proc/processor.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_calibration_parsers/parse.h>

#include "bag_tools/enhanced_stereo_image_saver.h"
#include "bag_tools/stereo_bag_processor.h"


bool stringToBool(const std::string& s) 
{
    if (s == "true" || s == "True" || s == "TRUE" || s == "1") 
        return true;
    return false;
}


int main(int argc, char** argv)
{
  if (argc < 11)
  {
    std::cout << "Saves rectified color images from raw and camera info input. It is possible to decimate images if the `decimation` parameter is greater than one. It is possible to enhance images with dehaze and clahe algorithms." << std::endl;
    std::cout << "Usage: " << argv[0] << " OUT_DIR FILETYPE STEREO_BASE_TOPIC LEFT_CAMERA_NAME RIGHT_CAMERA_NAME DECIMATION APPLY_DEHAZE APPLY_CLAHE CLIP_LIMIT BAGFILE [BAGFILE...]" << std::endl;
    std::cout << "  Example: " << argv[0] << " /tmp jpg /stereo_down /left /right 1 1 1 2 bag1.bag bag2.bag" << std::endl;
    return 0;
  }

  // Parsing.
  char *p;
  bool enable_dehaze, enable_clahe;
  std::string out_dir(argv[1]);
  std::string filetype(argv[2]);
  std::string base_topic(argv[3]);
  std::string l_cam_name(argv[4]);
  std::string r_cam_name(argv[5]);
  long decimation_long = std::strtol(argv[6], &p, 10);
  int decimation = static_cast<int>(decimation_long);
  enable_dehaze = stringToBool(argv[7]);
  enable_clahe = stringToBool(argv[8]);
  double clip_limit = std::stod(argv[9]);
  
  ros::Time::init();

  EnhancedStereoImageSaver saver(out_dir, filetype, l_cam_name, r_cam_name, enable_dehaze, enable_clahe, decimation, clip_limit);
  bag_tools::StereoBagProcessor processor(base_topic, l_cam_name, r_cam_name);
  processor.registerCallback(boost::bind(&EnhancedStereoImageSaver::save, &saver, _1, _2, _3, _4));

  for (int i = 10; i < argc; ++i)
    processor.processBag(argv[i]);

  return 0;
}