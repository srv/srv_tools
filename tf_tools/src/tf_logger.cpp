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
#include <tf/transform_listener.h>
#include <string>
#include <iostream>
#include <fstream>

std::ostream& operator<<(std::ostream& os, const tf::Transform& transform)
{
  using std::setw;
  tf::Vector3 origin = transform.getOrigin();
  tf::Quaternion quat = transform.getRotation();
  os << setw(10) << origin.x() << " " << setw(10) << origin.y() << " " << setw(10) << origin.z() << " ";
  os << setw(10) << quat.x() << " " << setw(10) << quat.y() << " " << setw(10) << quat.z() << " " << setw(10) << quat.w();
  return os;
}

void printHeader(std::ostream& os, const std::string& frame_id)
{
  os << frame_id << ".x " << frame_id << ".y " << frame_id << ".z "
     << frame_id << ".qx " << frame_id << ".qy " << frame_id << ".qz " << frame_id << ".qw ";
}

struct FramePair
{
  FramePair(const std::string& rframe, const std::string& lframe) :
    reference_frame(rframe), log_frame(lframe)
  {}
  std::string reference_frame;
  std::string log_frame;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_logger");

  if (argc < 4 || argc % 2 != 0)
  {
    std::cerr << "Wrong number of arguments." << std::endl;
    std::cerr << "USAGE: " << argv[0] 
      << " FREQUENCY REF_FRAME LOG_FRAME [REF_FRAME LOG_FRAME ...]" << std::endl;
    std::cerr << "The transforms should be published at a rate >= 2*FREQUENCY " << std::endl;
    return 1;
  }
    
  float frequency = atof(argv[1]);

  std::vector<FramePair> frame_pairs;

  std::cout << "#timestamp ";
  for (int i = 2; i < argc; i+=2)
  {
    std::string ref_frame(argv[i]);
    std::string log_frame(argv[i+1]);
    printHeader(std::cout, log_frame);
    frame_pairs.push_back(FramePair(ref_frame, log_frame));
  }
  std::cout << std::endl;
  
  ros::Duration cache_time(10);
  tf::TransformListener tf_listener(cache_time);
  
  ros::Rate rate(frequency);
  while (ros::ok())
  {
    rate.sleep();
    if (!ros::Time::isValid()) continue; // wait for clock
    ros::Time requested_time(ros::Time::now() - ros::Duration(2 / frequency));
    std::vector<tf::StampedTransform> transforms(frame_pairs.size());
    try
    {
      // first look up all transforms, this could throw exceptions
      for (size_t i = 0; i < frame_pairs.size(); ++i)
      {
        ros::Duration timeout(2 / frequency);
        ros::Duration polling_sleep_duration(0.5 / frequency);
        tf_listener.waitForTransform(
            frame_pairs[i].reference_frame, 
            frame_pairs[i].log_frame, requested_time,
            timeout, polling_sleep_duration);
        tf::StampedTransform transform;
        tf_listener.lookupTransform(
            frame_pairs[i].reference_frame, 
            frame_pairs[i].log_frame, requested_time, transforms[i]);
      }
      // no exception was thrown, print out set
      std::cout << requested_time.toNSec() << " ";
      for (size_t i = 0; i < transforms.size(); ++i)
      {
        std::cout << transforms[i] << " ";
      }
      std::cout << std::endl;
    }
    catch (const tf::TransformException& e)
    {
      std::cerr << "TransformException caught: " << e.what() << std::endl;
    }
  }
  return 0;
}

