#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sys/stat.h>

/**
 * Stores incoming point clouds in a map transforming
 * each cloud to a global fixed frame using tf.
 */
class PointCloudMapper
{
public:

  typedef pcl::PointXYZRGB                  Point;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  PointCloudMapper() :
    nh_(), nh_priv_("~")
  {
    // Read the parameters from the parameter server (set defaults)
    nh_priv_.param("graph_vertices_file", graph_vertices_file_, std::string("graph_vertices.txt"));
    nh_priv_.param("graph_blocking_file", graph_blocking_file_, std::string(".block.txt"));
    nh_priv_.param("x_filter_min", x_filter_min_, -2.0);
    nh_priv_.param("x_filter_max", x_filter_max_, 2.0);
    nh_priv_.param("y_filter_min", y_filter_min_, -2.0);
    nh_priv_.param("y_filter_max", y_filter_max_, 2.0);
    nh_priv_.param("z_filter_min", z_filter_min_, 0.2);
    nh_priv_.param("z_filter_max", z_filter_max_, 2.0);
    nh_priv_.param("voxel_size", voxel_size_, 0.1);
    nh_priv_.param("mean_k", mean_k_, 50);
    nh_priv_.param("std_dev_thresh", std_dev_thresh_, 1.0);
    nh_priv_.param("filter_map", filter_map_, true);

    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("input", 10, &PointCloudMapper::callback, this);
    bool latched = true;
    cloud_pub_ = nh_priv_.advertise<PointCloud>("output", 1, latched);
    pub_timer_ = nh_.createWallTimer(ros::WallDuration(3.0), 
                              &PointCloudMapper::timerCallback, this);
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    // Copy the cloud message
    sensor_msgs::PointCloud2 cur_cloud = *cloud;

    // Filter map if required
    if(filter_map_)
    {
      // Get the cloud
      PointCloud pcl_cloud;
      pcl::fromROSMsg(cur_cloud, pcl_cloud);

      // Filter
      PointCloud::Ptr cloud_downsampled = filter(pcl_cloud.makeShared());
      pcl_cloud = *cloud_downsampled;

      // Convert again to 
      pcl::toROSMsg(pcl_cloud, cur_cloud);
      
    }
    // Insert the cloud into the list
    pointcloud_list_.push_back(cur_cloud);
  }

  void timerCallback(const ros::WallTimerEvent& event)
  {
    // Detect subscribers
    if (cloud_pub_.getNumSubscribers() > 0)
    {
      // Initialize point cloud
      PointCloud accumulated_cloud;

      // Wait until the blocking file disappear
      while(fileExists(graph_blocking_file_.c_str()))
      {
        ROS_WARN_THROTTLE(2, "[PointCloudMapper:] Graph blocking file detected, waiting...");
        ros::Duration(0.5).sleep();
      }
      ROS_INFO("[PointCloudMapper:] Processing SLAM graph and pointclouds.");

      // Read the file
      std::vector<OdomMsg> graph_vertices;
      std::ifstream file(graph_vertices_file_.c_str());
      for(std::string line; std::getline(file, line);)
      {
        // Get the line values
        std::string s;
        std::istringstream f(line);
        std::vector<std::string> line_values;
        while (std::getline(f, s, ','))
          line_values.push_back(s);

        // Convert string values to real
        if(line_values.size() == 12)
        {
          OdomMsg odom_msg;

          std::ostringstream s_t, s_x, s_y, s_z, s_qx, s_qy, s_qz, s_qw;
          s_t << std::fixed << std::setprecision(9) << line_values.at(0);
          s_x << std::fixed << std::setprecision(7) << line_values.at(5);
          s_y << std::fixed << std::setprecision(7) << line_values.at(6);
          s_z << std::fixed << std::setprecision(7) << line_values.at(7);
          s_qx << std::fixed << std::setprecision(7) << line_values.at(8);
          s_qy << std::fixed << std::setprecision(7) << line_values.at(9);
          s_qz << std::fixed << std::setprecision(7) << line_values.at(10);
          s_qw << std::fixed << std::setprecision(7) << line_values.at(11);

          odom_msg.timestamp = boost::lexical_cast<double>(s_t.str());
          odom_msg.id = boost::lexical_cast<int>(line_values.at(1));
          odom_msg.fixed_frame = line_values.at(3);
          odom_msg.base_link_frame = line_values.at(4);
          odom_msg.x = boost::lexical_cast<double>(s_x.str());
          odom_msg.y = boost::lexical_cast<double>(s_y.str());
          odom_msg.z = boost::lexical_cast<double>(s_z.str());
          odom_msg.qx = boost::lexical_cast<double>(s_qx.str());
          odom_msg.qy = boost::lexical_cast<double>(s_qy.str());
          odom_msg.qz = boost::lexical_cast<double>(s_qz.str());
          odom_msg.qw = boost::lexical_cast<double>(s_qw.str());
          graph_vertices.push_back(odom_msg);
        }
        else
        {
          ROS_WARN_STREAM("[PointCloudMapper:] Line not processed: " << line);
        }
      }

      // Transform the point clouds
      for (unsigned int i=0; i<pointcloud_list_.size(); i++)
      {
        // Get the current cloud
        sensor_msgs::PointCloud2 cloud = pointcloud_list_.at(i);
        double cloud_time = cloud.header.stamp.toSec();

        // Search the corresponding timestamp in the graph
        int idx_graph = -1;
        double min_time_diff = graph_vertices[0].timestamp;
        for (unsigned int j=0; j<graph_vertices.size(); j++)
        {
          double time_diff = fabs(graph_vertices[j].timestamp - cloud_time);
          if(time_diff < min_time_diff)
          {
            min_time_diff = time_diff;
            idx_graph = j;
          }
        }

        // Found sync?
        double eps = 1e-1;
        if (min_time_diff < eps)
        {
          ROS_DEBUG_STREAM("[PointCloudMapper:] Time sync found between pointcloud " << 
                            i << " and graph vertex " << idx_graph << ".");

          // Build the tf
          tf::Vector3 t(graph_vertices[idx_graph].x, 
                        graph_vertices[idx_graph].y,
                        graph_vertices[idx_graph].z);
          tf::Quaternion q(graph_vertices[idx_graph].qx,
                           graph_vertices[idx_graph].qy,
                           graph_vertices[idx_graph].qz,
                           graph_vertices[idx_graph].qw);
          tf::Transform transform(q, t);

          // Transform pointcloud
          PointCloud pcl_cloud, transformed_cloud;
          pcl::fromROSMsg(pointcloud_list_.at(i), pcl_cloud);
          pcl_ros::transformPointCloud(pcl_cloud, transformed_cloud, transform);

          // Accumulate points
          accumulated_cloud += transformed_cloud;
        }
      }

      // If points...
      if(accumulated_cloud.points.size() > 0)
        cloud_pub_.publish(accumulated_cloud);
      else
        ROS_INFO("[PointCloudMapper:] Nothing to add...");
    }
  }

  PointCloud::Ptr filter(PointCloud::Ptr cloud)
  {
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

    // Statistical outlier removal
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(cloud_downsampled_ptr);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_dev_thresh_);
    sor.filter(*cloud_downsampled_ptr);

    return cloud_downsampled_ptr;
  }

  bool fileExists(const char* filename) 
  {
    struct stat info;
    int ret = -1;
   
    //get the file attributes
    ret = stat(filename, &info);
    if(ret == 0)
      return true;
    else 
      return false;
  }

private:
  struct OdomMsg 
  {
    double timestamp;
    int id;
    std::string fixed_frame;
    std::string base_link_frame;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
  };

  // Node parameters
  std::string graph_vertices_file_;
  std::string graph_blocking_file_;

  // Filter parameters
  double x_filter_min_;
  double x_filter_max_;
  double y_filter_min_;
  double y_filter_max_;
  double z_filter_min_;
  double z_filter_max_;
  double voxel_size_;
  double std_dev_thresh_;
  int mean_k_;
  bool filter_map_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  ros::WallTimer pub_timer_;

  std::vector<sensor_msgs::PointCloud2> pointcloud_list_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_mapper");
  PointCloudMapper mapper;
  ros::spin();
  return 0;
}

