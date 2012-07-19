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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <stereo_image_proc/processor.h>
#include <cv_bridge/cv_bridge.h>

#include <bag_tools/stereo_bag_processor.h>

class StereoImageProcessor
{
public:
  StereoImageProcessor(const std::string& base_topic, const std::string& out_bag, int processing_flags) :
    stereo_base_topic_(base_topic),
    flags_(processing_flags)
  {
    std::cout << "Opening bagfile '" << out_bag << "' for writing." << std::endl;
    bag_.open(out_bag, rosbag::bagmode::Write);
  }

  ~StereoImageProcessor()
  {
    std::cout << "Closing bagfile." << std::endl;
    bag_.close();
  }

  void process(const sensor_msgs::Image::ConstPtr &l_img, 
               const sensor_msgs::Image::ConstPtr &r_img, 
               const sensor_msgs::CameraInfo::ConstPtr &l_info,
               const sensor_msgs::CameraInfo::ConstPtr &r_info)
  {
    model_.fromCameraInfo(l_info, r_info);
    stereo_image_proc::StereoImageSet image_set;
    processor_.process(l_img, r_img, model_, image_set, flags_);

    bag_.write(stereo_base_topic_ + "/left/camera_info",
        l_info->header.stamp, l_info);
    bag_.write(stereo_base_topic_ + "/right/camera_info",
        r_info->header.stamp, r_info);

    if (flags_ & stereo_image_proc::StereoProcessor::LEFT_MONO)
    {
      sensor_msgs::Image::Ptr msg = createMsg(
          l_img->header, 
          image_set.left.color_encoding, 
          image_set.left.mono);
      bag_.write(stereo_base_topic_ + "/left/image_mono", 
          l_img->header.stamp, msg);
    }
    if (flags_ & stereo_image_proc::StereoProcessor::LEFT_COLOR)
    {
      sensor_msgs::Image::Ptr msg = createMsg(
          l_img->header, 
          image_set.left.color_encoding, 
          image_set.left.color);
      bag_.write(stereo_base_topic_ + "/left/image_color", 
          l_img->header.stamp, msg);
    }
    if (flags_ & stereo_image_proc::StereoProcessor::LEFT_RECT)
    {
      sensor_msgs::Image::Ptr msg = createMsg(
          l_img->header, 
          image_set.left.color_encoding, 
          image_set.left.rect);
      bag_.write(stereo_base_topic_ + "/left/image_rect", 
          l_img->header.stamp, msg);
    }
    if (flags_ & stereo_image_proc::StereoProcessor::LEFT_RECT_COLOR)
    {
      sensor_msgs::Image::Ptr msg = createMsg(
          l_img->header, 
          image_set.left.color_encoding, 
          image_set.left.rect_color);
      bag_.write(stereo_base_topic_ + "/left/image_rect_color", 
          l_img->header.stamp, msg);
    }
    if (flags_ & stereo_image_proc::StereoProcessor::RIGHT_MONO)
    {
      sensor_msgs::Image::Ptr msg = createMsg(
          r_img->header, 
          image_set.right.color_encoding, 
          image_set.right.mono);
      bag_.write(stereo_base_topic_ + "/right/image_mono", 
          r_img->header.stamp, msg);
    }
    if (flags_ & stereo_image_proc::StereoProcessor::RIGHT_COLOR)
    {
      sensor_msgs::Image::Ptr msg = createMsg(
          r_img->header, 
          image_set.right.color_encoding, 
          image_set.right.color);
      bag_.write(stereo_base_topic_ + "/right/image_color", 
          r_img->header.stamp, msg);
    }
    if (flags_ & stereo_image_proc::StereoProcessor::RIGHT_RECT)
    {
      sensor_msgs::Image::Ptr msg = createMsg(
          r_img->header, 
          image_set.right.color_encoding, 
          image_set.right.rect);
      bag_.write(stereo_base_topic_ + "/right/image_rect", 
          r_img->header.stamp, msg);
    }
    if (flags_ & stereo_image_proc::StereoProcessor::RIGHT_RECT_COLOR)
    {
      sensor_msgs::Image::Ptr msg = createMsg(
          r_img->header, 
          image_set.right.color_encoding, 
          image_set.right.rect_color);
      bag_.write(stereo_base_topic_ + "/right/image_rect_color", 
          r_img->header.stamp, msg);
    }

    if (flags_ & stereo_image_proc::StereoProcessor::POINT_CLOUD2)
    {
      bag_.write(stereo_base_topic_ + "/points2",
          l_img->header.stamp, image_set.points2);
    }

  }

  sensor_msgs::Image::Ptr createMsg(
      const std_msgs::Header& header, 
      const std::string& encoding, 
      const cv::Mat& img)
  {
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = encoding;
    cv_image.image = img;
    return cv_image.toImageMsg();
  }
 
private:

  rosbag::Bag bag_;
  std::string stereo_base_topic_;
  int flags_;
  image_geometry::StereoCameraModel model_;
  stereo_image_proc::StereoProcessor processor_;

};
 
int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Takes raw images from an input bagfile, processes them and writes the output to a new bagfile." << std::endl;
    std::cout << "Usage: " << argv[0] << " INBAG STEREO_BASE_TOPIC OUTBAG" << std::endl;
    std::cout << "  Example: " << argv[0] << " bag.bag /stereo_forward bag_processed.bag" << std::endl;
    return 0;
  }

  std::string in_bag(argv[1]);
  std::string base_topic(argv[2]);
  std::string out_bag(argv[3]);
  
  int flags = 0;
  flags |= stereo_image_proc::StereoProcessor::LEFT_RECT_COLOR;
  flags |= stereo_image_proc::StereoProcessor::RIGHT_RECT_COLOR;
  flags |= stereo_image_proc::StereoProcessor::POINT_CLOUD2;
  StereoImageProcessor image_processor(base_topic, out_bag, flags);
  bag_tools::StereoBagProcessor bag_processor(base_topic);
  bag_processor.registerCallback(boost::bind(&StereoImageProcessor::process, &image_processor, _1, _2, _3, _4));

  bag_processor.processBag(in_bag);

  return 0;
}

