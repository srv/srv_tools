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
#include <fstream>
#include <iostream>

#include <ros/ros.h>

#include <boost/format.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_proc/processor.h>
#include <camera_calibration_parsers/parse.h>

#include <opencv2/highgui/highgui.hpp> // for cv::imwrite

#include <cv_bridge/cv_bridge.h>

#include <boost/filesystem.hpp>
#include <bag_tools/tum_bag_processor.h>
using namespace std;
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

    cv_bridge::CvImagePtr cv_ptr;
    std::string save_dir;
    if (img->encoding == "16UC1") 
    {
      try
      {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      cv_ptr->image.convertTo(cv_ptr->image, CV_8U, 255.0 / 4096.0);

      save_dir = save_dir_ +"/depth";
    }

    else if (img->encoding == "rgb8") 
    {
      try
      {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC3);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      cv::cvtColor( cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB );
      save_dir = save_dir_ + "/rgb";
    }

    else
    {
      image_proc::ImageSet output;
      image_geometry::PinholeCameraModel camera_model; // empty, we don't need it here
      if (!processor_.process(img, camera_model, output, image_proc::Processor::COLOR))
      {
        std::cerr << "ERROR Processing image" << std::endl;
        return;
      }
    }

    std::string filename = 
      boost::str(boost::format("%s/%lu.%s") 
          % save_dir 
          % img->header.stamp.toNSec()
          // % prefix_  
          % filetype_);
    if (!cv::imwrite(filename, cv_ptr->image))
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

class TUMImageSaver
{
public:
  TUMImageSaver(const std::string& save_dir, const std::string& filetype) :
    depth_saver_(save_dir, filetype, "depth_"),
    color_saver_(save_dir, filetype, "color_"), num_img_(1), save_dir_(save_dir + "/associate.txt" )
  {}

  void save(const sensor_msgs::Image::ConstPtr &depth_img, 
            const sensor_msgs::Image::ConstPtr &color_img, 
            const sensor_msgs::CameraInfo::ConstPtr &depth_info,
            const sensor_msgs::CameraInfo::ConstPtr &color_info)
  {
    depth_saver_.save(depth_img, depth_info);
    color_saver_.save(color_img, color_info);

    std::ofstream outputFile_;  
    outputFile_.open(save_dir_.c_str(), std::ios::app);
    outputFile_ << num_img_ << "\t";
    outputFile_ << "rgb/" << color_img->header.stamp.toNSec() << ".png"<<"\t";
    outputFile_ << num_img_ << "\t";
    outputFile_ << "depth/" << depth_img->header.stamp.toNSec() << ".png"<< "\n";


    outputFile_.close();
    num_img_++;
  }

private:

  ImageSaver depth_saver_, color_saver_;
  int num_img_;
  std::string save_dir_;

};
 
int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: " << argv[0] << " OUT_DIR FILETYPE STEREO_BASE_TOPIC BAGFILE [BAGFILE...]" << std::endl;
    std::cout << "  Example: " << argv[0] << " /tmp png /camera bag1.bag bag2.bag" << std::endl;
    return 0;
  }

  std::string out_dir(argv[1]);
  std::string filetype(argv[2]);
  std::string base_topic(argv[3]);

  std::string out_txt_file = out_dir + "/associate.txt" ;

  // Create a folder if doesn't exist
  boost::filesystem::path dir(out_dir);

  if (!(boost::filesystem::exists(dir))) {
    std::cout << dir << " Doesn't Exists" << std::endl;

    if (boost::filesystem::create_directory(dir))
      std::cout << ".... Folder Successfully Created !" << std::endl;
  }

    //-- If associate file exist, clear the file data
  std::ifstream outputFile_(out_txt_file.c_str());
  if (outputFile_.good()) {
    std::cout << " Clearing data of Associations" << std::endl;    
    std::ofstream outputFile_(out_txt_file.c_str());
    outputFile_.close( );
  }

  // Create two folders
  boost::filesystem::path dir_rgb(out_dir + "/rgb");
  boost::filesystem::path dir_depth(out_dir + "/depth");


  if (!(boost::filesystem::exists(dir_rgb))) {
    std::cout << "Doesn't Exists" << std::endl;

    if (boost::filesystem::create_directory(dir_rgb))
      if (boost::filesystem::create_directory(dir_depth))
      std::cout << "....Successfully Created RGB and Depth folders !" << std::endl;
  }
  else 
  {
    std::cerr << " ERROR: This file has already data inside, please select another folder." << std::endl;
    return 0;
  }
  
  ros::Time::init();

  TUMImageSaver saver(out_dir, filetype);
  bag_tools::TUMBagProcessor processor(base_topic);
  processor.registerCallback(boost::bind(&TUMImageSaver::save, saver, _1, _2, _3, _4));

  for (int i = 4; i < argc; ++i)
    processor.processBag(argv[i]);

  return 0;
}

