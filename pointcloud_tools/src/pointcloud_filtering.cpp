#include <ros/ros.h>

#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB      Point;
typedef pcl::PointCloud<Point>PointCloud;

class PointCloudFiltering {

  // ROS properties
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber point_cloud_sub_;

  // Publisher to send out the filtered point cloud
  ros::Publisher point_cloud_filtered_;

  // Filter parameters
  double x_filter_min_;
  double x_filter_max_;
  double y_filter_min_;
  double y_filter_max_;
  double z_filter_min_;
  double z_filter_max_;
  double voxel_size_;
  int mean_k_;
  double std_dev_thresh_;

  bool apply_xyz_limits_;
  bool apply_voxel_grid_;
  bool apply_outlier_removal_;

public:

  /**
   * Class constructor
   */
  PointCloudFiltering() : nh_private_("~")
  {
    // Read the parameters from the parameter server (set defaults)
    nh_private_.param("apply_xyz_limits", apply_xyz_limits_, true);
    nh_private_.param("apply_voxel_grid", apply_voxel_grid_, true);
    nh_private_.param("apply_outlier_removal", apply_outlier_removal_, false);
    nh_private_.param("x_filter_min", x_filter_min_, -3.0);
    nh_private_.param("x_filter_max", x_filter_max_, 3.0);
    nh_private_.param("y_filter_min", y_filter_min_, -3.0);
    nh_private_.param("y_filter_max", y_filter_max_, 3.0);
    nh_private_.param("z_filter_min", z_filter_min_, 0.2);
    nh_private_.param("z_filter_max", z_filter_max_, 3.0);
    nh_private_.param("voxel_size", voxel_size_, 0.01);
    nh_private_.param("mean_k", mean_k_, 50);
    nh_private_.param("std_dev_thresh", std_dev_thresh_, 1.0);

    // Subscription to the point cloud result from stereo_image_proc
    point_cloud_sub_ = nh_.subscribe<PointCloud>(
      "input",
      1,
      &PointCloudFiltering::
      pointCloudCb,
      this);

    // Declare the point cloud filtered topic
    point_cloud_filtered_ = nh_private_.advertise<PointCloud>("output", 1);
  }

  /**
   * Callback executed when a point cloud is recieved from topic "points2".
   */
  void pointCloudCb(const PointCloud::ConstPtr& point_cloud)
  {
    PointCloud cloud = *point_cloud;

    // Filter
    PointCloud::Ptr cloud_downsampled = filter(cloud.makeShared());

    // Publish the filtered point cloud
    point_cloud_filtered_.publish(cloud_downsampled);
  }

  /**
   * Function to downsample the point cloud using Z-filtering (by range) and
   *voxel grid.
   */
  PointCloud::Ptr filter(PointCloud::Ptr cloud)
  {
    // NAN and limit filtering
    PointCloud::Ptr cloud_filtered_ptr(new PointCloud);
    pcl::PassThrough<Point> pass;

    if (apply_xyz_limits_)
    {
      // X-filtering
      pass.setFilterFieldName("x");
      pass.setFilterLimits(x_filter_min_, x_filter_max_);
      pass.setInputCloud(cloud);
      pass.filter(*cloud_filtered_ptr);

      // Y-filtering
      pass.setFilterFieldName("y");
      pass.setFilterLimits(y_filter_min_, y_filter_max_);
      pass.setInputCloud(cloud_filtered_ptr);
      pass.filter(*cloud_filtered_ptr);

      // Z-filtering
      pass.setFilterFieldName("z");
      pass.setFilterLimits(z_filter_min_, z_filter_max_);
      pass.setInputCloud(cloud_filtered_ptr);
      pass.filter(*cloud_filtered_ptr);
    }
    else
    {
      cloud_filtered_ptr = cloud;
    }

    // Downsampling using voxel grid
    
    PointCloud::Ptr cloud_downsampled_ptr(new PointCloud);

    if (apply_voxel_grid_)
    {
      pcl::VoxelGrid<Point> grid;
      grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      grid.setDownsampleAllData(true);
      grid.setInputCloud(cloud_filtered_ptr);
      grid.filter(*cloud_downsampled_ptr);
    }
    else
    {
      cloud_downsampled_ptr = cloud_filtered_ptr;
    }
    
    // Statistical outlier removal
    PointCloud::Ptr cloud_outlier_ptr(new PointCloud);

    if (apply_outlier_removal_)
    {
      pcl::StatisticalOutlierRemoval<Point> sor;
      sor.setInputCloud(cloud_downsampled_ptr);
      sor.setMeanK(mean_k_);
      sor.setStddevMulThresh(std_dev_thresh_);
      sor.filter(*cloud_outlier_ptr);
    }
    else
    {
      cloud_outlier_ptr = cloud_downsampled_ptr;
    }    

    return cloud_outlier_ptr;
  }
};

/**
 * Main entry point of the code
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_filtering");
  PointCloudFiltering node;
  ros::spin();
  return 0;
}

