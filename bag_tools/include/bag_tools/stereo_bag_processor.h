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
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/progress.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


namespace bag_tools
{

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    this->signalMessage(msg);
  }
};

class StereoBagProcessor
{

public:
  StereoBagProcessor(const std::string& stereo_base_topic) :
    stereo_base_topic_(stereo_base_topic),
    sync_(l_img_sub_, r_img_sub_, l_info_sub_, r_info_sub_, 25)
  {
    ros::Time::init();
  }

  template<class C>
  void registerCallback(const C& callback)
  {
    sync_.registerCallback(callback);
  }

  /**
   * Processes given bagfile, calls registered callback function when
   * a synchronized stereo pair with camera infos is found.
   */
  void processBag(const std::string &filename)
  {

    // Image topics to load
    std::string l_cam = stereo_base_topic_ + "/left";
    std::string r_cam = stereo_base_topic_ + "/right";
    std::string l_cam_image = l_cam + "/image_raw";
    std::string r_cam_image = r_cam + "/image_raw";
    std::string l_cam_info = l_cam + "/camera_info";
    std::string r_cam_info = r_cam + "/camera_info";

    std::vector<std::string> topics;
    topics.push_back(l_cam_image);
    topics.push_back(r_cam_image);
    topics.push_back(l_cam_info);
    topics.push_back(r_cam_info);

    std::cout << "Starting processing " << filename << ", ";
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int num_messages = view.size();
    std::cout << num_messages << " messages to process." << std::endl;

    // Load all messages
    boost::progress_display show_progress(num_messages);
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image))
      {
        sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
        if (l_img != NULL)
          l_img_sub_.newMessage(l_img);
      }

      if (m.getTopic() == r_cam_image || ("/" + m.getTopic() == r_cam_image))
      {
        sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
        if (r_img != NULL)
          r_img_sub_.newMessage(r_img);
      }

      if (m.getTopic() == l_cam_info || ("/" + m.getTopic() == l_cam_info))
      {
        sensor_msgs::CameraInfo::ConstPtr l_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (l_info != NULL)
          l_info_sub_.newMessage(l_info);
      }

      if (m.getTopic() == r_cam_info || ("/" + m.getTopic() == r_cam_info))
      {
        sensor_msgs::CameraInfo::ConstPtr r_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (r_info != NULL)
          r_info_sub_.newMessage(r_info);
      }
      ++show_progress;
    }
    bag.close();
    std::cout << "Finished processing " << filename << std::endl;
  }

private:

  // Fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> l_img_sub_, r_img_sub_;
  BagSubscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;

  std::string stereo_base_topic_;

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_;

};

} // end of namespace

