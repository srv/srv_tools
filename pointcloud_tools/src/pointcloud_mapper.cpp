#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

/**
 * Stores incoming point clouds in a map transforming
 * each cloud to a global fixed frame using tf.
 */
class PointCloudMapper
{
public:

  typedef pcl::PointXYZRGB      Point;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  PointCloudMapper() :
    nh_(), nh_priv_("~")
  {
    nh_priv_.param("fixed_frame", fixed_frame_, std::string("/map"));
    // Read the parameters from the parameter server (set defaults)
    nh_priv_.param("x_filter_min", x_filter_min_, -30.0);
    nh_priv_.param("x_filter_max", x_filter_max_, 30.0);
    nh_priv_.param("y_filter_min", y_filter_min_, -30.0);
    nh_priv_.param("y_filter_max", y_filter_max_, 30.0);
    nh_priv_.param("z_filter_min", z_filter_min_, 2.0);
    nh_priv_.param("z_filter_max", z_filter_max_, 30.0);
    nh_priv_.param("voxel_size", voxel_size_, 0.1);
    nh_priv_.param("filter_map", filter_map_, false);

    cloud_sub_ = nh_.subscribe<PointCloud>("input", 10, &PointCloudMapper::callback, this);
    bool latched = true;
    cloud_pub_ = nh_priv_.advertise<PointCloud>("output", 1, latched);
    pub_timer_ = nh_.createTimer(ros::Duration(10.0), &PointCloudMapper::publishCallback, this);
  }

  void callback(const PointCloud::ConstPtr& cloud)
  {
    ROS_INFO_STREAM("received cloud with " << cloud->points.size() << " points.");
    PointCloud transformed_cloud;
    bool success = pcl_ros::transformPointCloud(fixed_frame_, *cloud, transformed_cloud, tf_listener_);
    if (success)
    {
      accumulated_cloud_ += transformed_cloud;
    }
    else
    {
      ROS_ERROR("Could not transform point cloud to %s", fixed_frame_.c_str());
    }

    if(filter_map_){
      PointCloud::Ptr cloud_downsampled = filter(accumulated_cloud_.makeShared());
      accumulated_cloud_ = *cloud_downsampled;
    }


    ROS_INFO_STREAM("Map has " << accumulated_cloud_.points.size() << " points.");
  }

  void publishCallback(const ros::TimerEvent&)
  {
    if (cloud_pub_.getNumSubscribers() > 0)
    {
      cloud_pub_.publish(accumulated_cloud_);
    }
  }

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

private:
  // Filter parameters
  double x_filter_min_;
  double x_filter_max_;
  double y_filter_min_;
  double y_filter_max_;
  double z_filter_min_;
  double z_filter_max_;
  double voxel_size_;
  bool filter_map_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  ros::Timer pub_timer_;

  std::string fixed_frame_;
  tf::TransformListener tf_listener_;
  PointCloud accumulated_cloud_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_mapper");
  PointCloudMapper mapper;
  ros::spin();
  return 0;
}

