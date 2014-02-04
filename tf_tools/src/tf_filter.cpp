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
#include <tf/transform_broadcaster.h>
#include <string>
#include <iostream>

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_filter");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  std::string reference_frame_id;
  std::string child_frame_id;
  double frequency, ramp_wwf;
  int num_samples_wwf, wwf_index;
  bool full_arrays = false;

  nhp.param("reference_frame_id", reference_frame_id, std::string("/base_link"));
  nhp.param("child_frame_id", child_frame_id, std::string("/camera"));
  nhp.param("frequency", frequency, 100.0);

  // Use Weighted Window Filter
  nhp.param("ramp_wwf", ramp_wwf, 0.5);
  nhp.param("num_samples_wwf", num_samples_wwf, 10);
 
  ROS_INFO("TF Filter initiated.\n\tPARAMETERS: \
            \n\t * Reference frame ID: %s \
            \n\t * Child frame ID:     %s \
            \n\t * Frequency:          %f \
            \n\t * Ramp WWF:           %f \
            \n\t * Num samples WWF:    %d", 
            reference_frame_id.c_str(),
            child_frame_id.c_str(),
            frequency,
            ramp_wwf, 
            num_samples_wwf);

  double * x_array, * y_array, * z_array;
  double * sR_array, * cR_array, * sP_array, * cP_array, * sY_array, * cY_array;
  double * t_array, * w_array;

  //fill circular arrays with zeros
  wwf_index = 0;
  x_array  = new double[num_samples_wwf];
  y_array  = new double[num_samples_wwf];
  z_array  = new double[num_samples_wwf];
  sR_array = new double[num_samples_wwf]; // sin(roll) array
  cR_array = new double[num_samples_wwf]; // cos(roll) array
  sP_array = new double[num_samples_wwf]; // sin(pitch) array
  cP_array = new double[num_samples_wwf]; // cos(pitch) array
  sY_array = new double[num_samples_wwf]; // sin(yaw) array
  cY_array = new double[num_samples_wwf]; // cos(yaw) array
  t_array  = new double[num_samples_wwf]; //times used for the ramp
  w_array  = new double[num_samples_wwf]; //weights
  for(int i = 0; i< num_samples_wwf; i++){
    x_array[i] = 0.0;
    y_array[i] = 0.0;
    z_array[i] = 0.0;
    sR_array[i] = 0.0;
    cR_array[i] = 0.0;
    sP_array[i] = 0.0;
    cP_array[i] = 0.0;
    sY_array[i] = 0.0;
    cY_array[i] = 0.0;
    t_array[i] = 0.0; 
    w_array[i] = 0.0; 
  }

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  
  ros::Rate rate(frequency);
  while (ros::ok())
  {
    rate.sleep(); //TODO: here or at the end?

    // Wait for clock
    if (!ros::Time::isValid()) continue; 
    ros::Time requested_time = ros::Time(0); //(ros::Time::now() - ros::Duration(2 / frequency));
    tf::StampedTransform transform;
    try
    {
      // first look up transform, this could throw exceptions
      ros::Duration timeout(2 / frequency);
      ros::Duration polling_sleep_duration(0.5 / frequency);
      listener.waitForTransform(
          reference_frame_id, 
          child_frame_id, 
          requested_time,
          timeout, 
          polling_sleep_duration);
      listener.lookupTransform(
          reference_frame_id, 
          child_frame_id, 
          requested_time, 
          transform);

      ROS_DEBUG_STREAM("Received transform: " << transform);
      ROS_DEBUG_STREAM("With frame_id: " << transform.frame_id_ << " and child_id " << transform.child_frame_id_);

      // get position and orientation
      double roll, pitch, yaw;
      tf::Matrix3x3 rotm(transform.getRotation());
      rotm.getRPY(roll, pitch, yaw);

      x_array[wwf_index] = transform.getOrigin().x();
      y_array[wwf_index] = transform.getOrigin().y();
      z_array[wwf_index] = transform.getOrigin().z();
      sR_array[wwf_index] = sin(roll);
      cR_array[wwf_index] = cos(roll);
      sP_array[wwf_index] = sin(pitch);
      cP_array[wwf_index] = cos(pitch);
      sY_array[wwf_index] = sin(yaw);
      cY_array[wwf_index] = cos(yaw);

      // save ros time into circular array
      ros::Time now = transform.stamp_;
      t_array[wwf_index] = now.toSec();

      // compute weights for every element of the arrays
      // and leave weights of empty cells to zero
      int num_elem = num_samples_wwf;
      if(!full_arrays){
        num_elem = wwf_index + 1; 
      }
      double w_total = 0.0;
      for(int i = 0; i < num_elem; i++){
        double t_aux = t_array[i] - t_array[wwf_index]; //<=0
        w_array[i] = std::max(0.0,ramp_wwf*t_aux + 1);
        w_total += w_array[i];
      }

      
      double w_aux;
      double x, y, z, sinR, cosR, sinP, cosP, sinY, cosY;
      x = y = z = sinR = cosR = sinP = cosP = sinY = cosY = 0.0;

      // Compute final tf
      for(int i = 0; i < num_elem; i++){
        w_aux = w_array[i]/w_total;
        x += w_aux * x_array[i];
        y += w_aux * y_array[i];
        z += w_aux * z_array[i];
        sinR += w_aux * sR_array[i];
        cosR += w_aux * cR_array[i];
        sinP += w_aux * sP_array[i];
        cosP += w_aux * cP_array[i];
        sinY += w_aux * sY_array[i];
        cosY += w_aux * cY_array[i];
      }

      tf::Vector3 orig(x,y,z);

      roll = atan2(sinR, cosR);
      pitch = atan2(sinP, cosP);
      yaw = atan2(sinY, cosY);

      tf::Quaternion quat;
      quat.setRPY(roll, pitch, yaw);

      transform.setOrigin(orig);
      transform.setRotation(quat);

      // Set the output stamped transform and send it
      transform.child_frame_id_ = transform.child_frame_id_ + "_filtered";
      broadcaster.sendTransform(transform);

      // Increment the counter
      wwf_index++;
      if (wwf_index==num_samples_wwf){
        full_arrays = true;
        wwf_index = 0;
      }
    }
    catch (const tf::TransformException& e)
    {
      std::cerr << "TransformException caught: " << e.what() << std::endl;
    }
  }
  return 0;
}

