#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>

/**
 * Stores incoming point clouds in a map transforming
 * each cloud to a global fixed frame using tf.
 */
class PointCloudMapper
{
public:

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  PointCloudMapper() :
    nh_(), nh_priv_("~")
  {
    nh_priv_.param("fixed_frame", fixed_frame_, std::string("/map"));
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
    ROS_INFO_STREAM("Map has " << accumulated_cloud_.points.size() << " points.");
  }

  void publishCallback(const ros::TimerEvent&)
  {
    if (cloud_pub_.getNumSubscribers() > 0)
    {
      cloud_pub_.publish(accumulated_cloud_);
    }
  }

private:
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

