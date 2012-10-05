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

#include <ros/ros.h>

#include <boost/format.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_proc/processor.h>
#include <camera_calibration_parsers/parse.h>

#include <opencv2/highgui/highgui.hpp> // for cv::imwrite

#include <bag_tools/stereo_bag_processor.h>

/**
 * Saves rectified color images from raw and camera info input
 */
class ImageSaver
{
public:
  ImageSaver(const std::string& save_dir, const std::string& filetype, 
      const std::string& prefix) :
    save_dir_(save_dir), filetype_(filetype), prefix_(prefix), num_saved_(0)
  {}

  void save(const sensor_msgs::Image::ConstPtr &img, 
            const sensor_msgs::CameraInfo::ConstPtr &info)
  {
    camera_model_.fromCameraInfo(info);
    std::string calib_filename_ini = 
      boost::str(boost::format("%s/calibration_%scamera.ini") 
          % save_dir_ % prefix_ );
    std::string calib_filename_yaml = 
      boost::str(boost::format("%s/calibration_%scamera.yaml") 
          % save_dir_ % prefix_ );
    camera_calibration_parsers::writeCalibration(calib_filename_yaml, prefix_ + "camera", *info);
    image_proc::ImageSet output;
    if (!processor_.process(img, camera_model_, output, image_proc::Processor::RECT_COLOR))
    {
      std::cerr << "ERROR Processing image" << std::endl;
      return;
    }
    std::string filename = 
      boost::str(boost::format("%s/%s%lu.%s") 
          % save_dir_ 
          % prefix_ 
          % img->header.stamp.toNSec() 
          % filetype_);
    if (!cv::imwrite(filename, output.rect_color))
      ROS_ERROR_STREAM("ERROR Saving " << filename);
    else
    {
      ROS_DEBUG_STREAM("Saved " << filename);
      num_saved_++;
    }
  }

private:

  image_proc::Processor processor_;
  image_geometry::PinholeCameraModel camera_model_;
  std::string save_dir_;
  std::string filetype_;
  std::string prefix_;
  int num_saved_;

};

class StereoImageSaver
{
public:
  StereoImageSaver(const std::string& save_dir, const std::string& filetype) :
    l_saver_(save_dir, filetype, "left_"),
    r_saver_(save_dir, filetype, "right_")
  {}

  void save(const sensor_msgs::Image::ConstPtr &l_img, 
            const sensor_msgs::Image::ConstPtr &r_img, 
            const sensor_msgs::CameraInfo::ConstPtr &l_info,
            const sensor_msgs::CameraInfo::ConstPtr &r_info)
  {
    l_saver_.save(l_img, l_info);
    r_saver_.save(r_img, r_info);
  }

private:

  ImageSaver l_saver_, r_saver_;

};
 
int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: " << argv[0] << " OUT_DIR FILETYPE STEREO_BASE_TOPIC BAGFILE [BAGFILE...]" << std::endl;
    std::cout << "  Example: " << argv[0] << " /tmp jpg /stereo_down bag1.bag bag2.bag" << std::endl;
    return 0;
  }

  std::string out_dir(argv[1]);
  std::string filetype(argv[2]);
  std::string base_topic(argv[3]);
  
  ros::Time::init();

  StereoImageSaver saver(out_dir, filetype);
  bag_tools::StereoBagProcessor processor(base_topic);
  processor.registerCallback(boost::bind(&StereoImageSaver::save, saver, _1, _2, _3, _4));

  for (int i = 4; i < argc; ++i)
    processor.processBag(argv[i]);

  return 0;
}

