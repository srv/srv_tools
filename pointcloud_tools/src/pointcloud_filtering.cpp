#include <ros/ros.h>

#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

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

public:

  /**
   * Class constructor
   */
  PointCloudFiltering() : nh_private_("~")
  {
    // Read the parameters from the parameter server (set defaults)
    nh_private_.param("x_filter_min", x_filter_min_, -3.0);
    nh_private_.param("x_filter_max", x_filter_max_, 3.0);
    nh_private_.param("y_filter_min", y_filter_min_, -3.0);
    nh_private_.param("y_filter_max", y_filter_max_, 3.0);
    nh_private_.param("z_filter_min", z_filter_min_, 0.2);
    nh_private_.param("z_filter_max", z_filter_max_, 3.0);
    nh_private_.param("voxel_size", voxel_size_, 0.01);

    // Subscription to the point cloud result from stereo_image_proc
    point_cloud_sub_ = nh_.subscribe<PointCloud>(
      "points2",
      1,
      &PointCloudFiltering::
      pointCloudCb,
      this);

    // Declare the point cloud filtered topic
    point_cloud_filtered_ = nh_private_.advertise<PointCloud>("points2_filtered", 1);
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
    // Copy the point cloud
    PointCloud::Ptr cloud_ptr(new PointCloud);

    // NAN and limit filtering
    PointCloud::Ptr cloud_filtered_ptr(new PointCloud);
    pcl::PassThrough<Point> pass_;

    // X-filtering
    pass_.setFilterFieldName("x");
    pass_.setFilterLimits(x_filter_min_, x_filter_max_);
    pass_.setInputCloud(cloud);
    pass_.filter(*cloud_filtered_ptr);

    // Y-filtering
    pass_.setFilterFieldName("y");
    pass_.setFilterLimits(y_filter_min_, y_filter_max_);
    pass_.setInputCloud(cloud_filtered_ptr);
    pass_.filter(*cloud_filtered_ptr);

    // Z-filtering
    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(z_filter_min_, z_filter_max_);
    pass_.setInputCloud(cloud);
    pass_.filter(*cloud_filtered_ptr);

    // Downsampling using voxel grid
    pcl::VoxelGrid<Point> grid_;
    PointCloud::Ptr cloud_downsampled_ptr(new PointCloud);
    double plane_detection_voxel_size_ = voxel_size_;

    grid_.setLeafSize(plane_detection_voxel_size_,
                      plane_detection_voxel_size_,
                      plane_detection_voxel_size_);
    grid_.setDownsampleAllData(true);
    grid_.setInputCloud(cloud_filtered_ptr);
    grid_.filter(*cloud_downsampled_ptr);

    return cloud_downsampled_ptr;
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

