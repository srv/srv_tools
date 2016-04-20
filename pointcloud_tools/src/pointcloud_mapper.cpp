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

    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("input", 1, &PointCloudMapper::callback, this);
    cloud_pub_ = nh_priv_.advertise<sensor_msgs::PointCloud2>("output", 1, true);

    accumulated_cloud_.header.frame_id = fixed_frame_;

    ROS_INFO_STREAM("[PointCloudMapper params:\n" <<
                    "\t* Fixed frame:   " << fixed_frame_ << "\n" <<
                    "\t* Filter map:    " << filter_map_ << "\n" <<
                    "\t* Filter bounds: " << "( " << x_filter_min_ << ", " << y_filter_min_ << ", " << z_filter_min_ << ") and ( " << x_filter_max_ << ", " << y_filter_max_ << ", " << z_filter_max_ << ")" << "\n" <<
                    "\t* Voxel size:    " << voxel_size_);

    pub_timer_ = nh_.createWallTimer(ros::WallDuration(3.0), &PointCloudMapper::publishCallback, this);
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    PointCloud cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    ROS_INFO_STREAM("Received cloud with " << cloud.points.size() << " points.");

    PointCloud transformed_cloud;
    tf::StampedTransform transform;
    try {
      tf_listener_.waitForTransform(fixed_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(5.0));
      tf_listener_.lookupTransform(fixed_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, transform);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    pcl_ros::transformPointCloud(cloud, transformed_cloud, tf::Transform(transform));
    bool success = true;
    if (success)
    {
      accumulated_cloud_ += transformed_cloud;

      if(filter_map_)
      {
        PointCloud::Ptr cloud_downsampled = filter(accumulated_cloud_.makeShared());
        accumulated_cloud_ = *cloud_downsampled;
      }

      ROS_INFO_STREAM("Map has " << accumulated_cloud_.points.size() << " points.");

      // Publish the cloud
      if (cloud_pub_.getNumSubscribers() > 0)
        cloud_pub_.publish(accumulated_cloud_);
      last_pub_time_ = ros::WallTime::now();
    }
    else
    {
      ROS_ERROR("Could not transform point cloud to %s", fixed_frame_.c_str());
    }
  }

  void publishCallback(const ros::WallTimerEvent&)
  {
    // Publish the accumulated cloud if last publication was more than 5 seconds before.
    ros::WallDuration elapsed_time = ros::WallTime::now() - last_pub_time_;
    if (cloud_pub_.getNumSubscribers() > 0 && elapsed_time.toSec() > 5.0) {

      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(accumulated_cloud_, cloud_msg);
      cloud_pub_.publish(cloud_msg);
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

  ros::WallTimer pub_timer_;

  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  std::string fixed_frame_;
  tf::TransformListener tf_listener_;
  PointCloud accumulated_cloud_;
  ros::WallTime last_pub_time_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_mapper");
  PointCloudMapper mapper;
  ros::spin();
  return 0;
}

