/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pointcloud_online_viewer.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

// ROS core
#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
// PCL includes
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

using pcl::visualization::PointCloudColorHandlerGenericField;

typedef pcl::PointXYZ             Point;
typedef pcl::PointNormal          PointNormal;
typedef pcl::PointCloud<Point>    PointCloud;
typedef pcl::PointCloud<PointNormal>    PointCloudNormal;
typedef pcl::PointXYZRGB          PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

// Global data
sensor_msgs::PointCloud2ConstPtr cloud_, cloud_old_;
boost::mutex m_;
bool viewer_initialized_;
bool save_cloud_;
std::string pcd_filename_;
int counter_;
ros::WallTimer save_timer_;

PointCloud cloud_xyz_;
PointCloudNormal cloud_xyzn_;
PointCloudRGB cloud_xyz_rgb_;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  m_.lock ();
  ROS_INFO_STREAM("[PointCloudViewer:] Pointcloud received.");
  cloud_ = cloud;
  m_.unlock();
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym() == "space" && event.keyDown()) {
    ROS_INFO("[PointCloudViewer:] Saving pointcloud, please wait...");
    save_cloud_ = true;
  }
}

void updateVisualization()
{
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f               xyz_centroid;

  ros::WallDuration d(0.01);
  bool rgb = false;
  bool normal = false;
  std::vector<pcl::PCLPointField> fields;

  // Create the visualizer
  pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

  // Add a coordinate system to screen
  viewer.addCoordinateSystem(0.1);
  viewer.registerKeyboardCallback(&keyboardEventOccurred);

  while(true)
  {
    d.sleep();

    // If no cloud received yet, continue
    if(!cloud_ || cloud_->width<=0)
      continue;

    viewer.spinOnce(1);

    if(cloud_old_ == cloud_)
      continue;

    // Convert to PointCloud<T>
    m_.lock ();
    if(pcl::getFieldIndex(*cloud_, "rgb") != -1)
    {
      rgb = true;
      pcl::fromROSMsg(*cloud_, cloud_xyz_rgb_);
    }
    else if (pcl::getFieldIndex(*cloud_, "normal_x") != -1)
    {
      normal = true;
      pcl::fromROSMsg(*cloud_, cloud_xyzn_);
    }
    else
    {
      rgb = false;
      normal = false;
      pcl::fromROSMsg(*cloud_, cloud_xyz_);
      pcl::getFields(cloud_xyz_, fields);
    }
    cloud_old_ = cloud_;
    m_.unlock();

    // Delete the previous point cloud
    viewer.removePointCloud("cloud");

    // If no RGB data present, use a simpler white handler
    if(rgb && pcl::getFieldIndex(cloud_xyz_rgb_, "rgb", fields) != -1 &&
      cloud_xyz_rgb_.points[0].rgb != 0)
    {
      // Initialize the camera view
      if(!viewer_initialized_)
      {
        pcl::computeMeanAndCovarianceMatrix(cloud_xyz_rgb_, covariance_matrix, xyz_centroid);
        viewer.initCameraParameters();
        viewer.setCameraPosition(xyz_centroid(0), xyz_centroid(1), xyz_centroid(2)+3.0, 0, -1, 0);
        ROS_INFO_STREAM("[PointCloudViewer:] Point cloud rgb viewer camera initialized in: [" <<
          xyz_centroid(0) << ", " << xyz_centroid(1) << ", " << xyz_centroid(2)+3.0 << "]");
        viewer_initialized_ = true;
      }
      // Show the point cloud
      pcl::visualization::PointCloudColorHandlerRGBField<PointRGB> color_handler(
        cloud_xyz_rgb_.makeShared());
      viewer.addPointCloud(cloud_xyz_rgb_.makeShared(), color_handler, "cloud");
    }
    else if (normal && pcl::getFieldIndex(cloud_xyzn_, "normal_x", fields) != -1)
    {
      // Initialize the camera view
      if(!viewer_initialized_)
      {
        pcl::computeMeanAndCovarianceMatrix(cloud_xyzn_, covariance_matrix, xyz_centroid);
        viewer.initCameraParameters();
        viewer.setCameraPosition(xyz_centroid(0), xyz_centroid(1), xyz_centroid(2)+3.0, 0, -1, 0);
        ROS_INFO_STREAM("[PointCloudViewer:] Point cloud normal viewer camera initialized in: [" <<
          xyz_centroid(0) << ", " << xyz_centroid(1) << ", " << xyz_centroid(2)+3.0 << "]");
        viewer_initialized_ = true;
      }
      // Show the point cloud
      // Show the xyz point cloud
      PointCloudColorHandlerGenericField<PointNormal> color_handler (cloud_xyzn_.makeShared(), "z");
      if (!color_handler.isCapable ())
      {
        ROS_WARN_STREAM("[PointCloudViewer:] Cannot create curvature color handler!");
        pcl::visualization::PointCloudColorHandlerCustom<PointNormal> color_handler(
        cloud_xyzn_.makeShared(), 255, 0, 255);
      }
      viewer.addPointCloud<PointNormal>(cloud_xyzn_.makeShared(), color_handler, "cloud");
      // Delete the previous point cloud
      viewer.removePointCloud("normals");
      viewer.addPointCloudNormals<PointNormal>(cloud_xyzn_.makeShared(), 100, 0.02, "normals");
    }
    else
    {
      // Initialize the camera view
      if(!viewer_initialized_)
      {
        pcl::computeMeanAndCovarianceMatrix(cloud_xyz_, covariance_matrix, xyz_centroid);
        viewer.initCameraParameters();
        viewer.setCameraPosition(xyz_centroid(0), xyz_centroid(1), xyz_centroid(2)+3.0, 0, -1, 0);
        ROS_INFO_STREAM("[PointCloudViewer:] Point cloud viewer camera initialized in: [" <<
          xyz_centroid(0) << ", " << xyz_centroid(1) << ", " << xyz_centroid(2)+3.0 << "]");
        viewer_initialized_ = true;
      }

      // Show the xyz point cloud
      PointCloudColorHandlerGenericField<Point> color_handler (cloud_xyz_.makeShared(), "z");
      if (!color_handler.isCapable ())
      {
        ROS_WARN_STREAM("[PointCloudViewer:] Cannot create curvature color handler!");
        pcl::visualization::PointCloudColorHandlerCustom<Point> color_handler(
        cloud_xyz_.makeShared(), 255, 0, 255);
      }
      viewer.addPointCloud(cloud_xyz_.makeShared(), color_handler, "cloud");
    }

    counter_++;
  }
}

void saveCallback(const ros::WallTimerEvent&)
{
  if (!save_cloud_) return;

  if (cloud_xyz_rgb_.size() > 0)
  {
    // Remove outliers
    pcl::PointCloud<PointRGB>::Ptr cloud_filtered (new pcl::PointCloud<PointRGB>);
    pcl::StatisticalOutlierRemoval<PointRGB> sor;
    sor.setInputCloud (cloud_xyz_rgb_.makeShared());
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    if (pcl::io::savePCDFile(pcd_filename_, *cloud_filtered) == 0)
      ROS_INFO_STREAM("[PointCloudViewer:] Pointcloud saved into: " << pcd_filename_);
    else
      ROS_ERROR_STREAM("[PointCloudViewer:] Problem saving " << pcd_filename_.c_str());
    save_cloud_ = false;
  }

  if (cloud_xyz_.size() > 0)
  {
    // Remove outliers
    pcl::PointCloud<Point>::Ptr cloud_filtered (new pcl::PointCloud<Point>);
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud (cloud_xyz_.makeShared());
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    if (pcl::io::savePCDFile(pcd_filename_, *cloud_filtered) == 0)
      ROS_INFO_STREAM("[PointCloudViewer:] Pointcloud saved into: " << pcd_filename_);
    else
      ROS_ERROR_STREAM("[PointCloudViewer:] Problem saving " << pcd_filename_.c_str());
    save_cloud_ = false;
  }
}

// void sigIntHandler(int sig)
// {
//   exit(0);
// }

/* ---[ */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_viewer");//, ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  viewer_initialized_ = false;
  save_cloud_ = false;
  counter_ = 0;

  // Read parameters
  nh_priv.param("pcd_filename", pcd_filename_, std::string("pointcloud_file.pcd"));

  // Create a ROS subscriber
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  ROS_INFO("[PointCloudViewer:] Subscribing to %s for PointCloud2 messages...", nh.resolveName ("input").c_str ());

  // signal(SIGINT, sigIntHandler);
  boost::thread visualization_thread(&updateVisualization);
  save_timer_ = nh.createWallTimer(ros::WallDuration(1), &saveCallback);

  // Spin
  ros::spin();

  // Join, delete, exit
  visualization_thread.join();
  return (0);
}
/* ]--- */
