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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

namespace bag_tools
{

class ImageBagProcessor
{

public:

  typedef boost::function<void (const sensor_msgs::ImageConstPtr&)> CallbackType;
  typedef boost::function<void (const sensor_msgs::CompressedImageConstPtr&)> CallbackCompressedType;

  ImageBagProcessor(const std::string& image_topic) :
    image_topic_(image_topic)
  {
    ros::Time::init();
  }

  void registerCallback(const CallbackType& callback)
  {
    callback_ = callback;
  }

  void registerCompressedCallback(const CallbackCompressedType& callback)
  {
    callback_compressed_ = callback;
  }

  /**
   * Processes given bagfile, calls registered callback function when
   * a message is found.
   */
  void processBag(const std::string &filename)
  {
    // Image topics to load
    std::vector<std::string> topics;
    topics.push_back(image_topic_);

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
      if (m.getTopic() == image_topic_)
      {
        sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
        if (img_msg != NULL && callback_.empty() == false)
        {
          callback_(img_msg);
        }else
        {
          sensor_msgs::CompressedImageConstPtr compressed = m.instantiate<sensor_msgs::CompressedImage>();
          if(compressed != NULL && callback_compressed_.empty() == false)
          {
              callback_compressed_(compressed);
          }
        }
      }
      ++show_progress;
    }
    bag.close();
    std::cout << "Finished processing " << filename << std::endl;
  }

private:

  std::string image_topic_;
  CallbackType callback_;
  CallbackCompressedType callback_compressed_;

};

} // end of namespace

