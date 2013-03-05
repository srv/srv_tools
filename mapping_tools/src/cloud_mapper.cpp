#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <mapping_tools/GetCloud.h>

/**
 * Stores incoming point clouds in a map transforming
 * each cloud to a global fixed frame using tf.
 */
class CloudMapper
{
public:

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  CloudMapper() :
    nh_(), nh_priv_("~")
  {
    nh_priv_.param("fixed_frame", fixed_frame_, std::string("/map"));
    nh_priv_.param("frequency", freq_, 10.0);

    bool latched = true;
    cloud_sub_ = nh_.subscribe<PointCloud>("cloud", 10, &CloudMapper::callback, this);
    cloud_pub_ = nh_priv_.advertise<PointCloud>("accumulated_cloud", 1, latched);
	service_ = nh_priv_.advertiseService("get_cloud", &CloudMapper::get_cloud, this);
    
	pub_timer_ = nh_.createTimer(ros::Duration(freq_), &CloudMapper::publishCallback, this);
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

  bool get_cloud(mapping_tools::GetCloud::Request& req, mapping_tools::GetCloud::Response& res)
  {
	  pcl::toROSMsg(accumulated_cloud_, res.cloud);
	  return true;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  ros::ServiceServer service_;

  ros::Timer pub_timer_;

  std::string fixed_frame_;
  double freq_;
  tf::TransformListener tf_listener_;
  PointCloud accumulated_cloud_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_mapper");
  CloudMapper mapper;
  ros::spin();
  return 0;
}

