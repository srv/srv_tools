#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ             Point;
typedef pcl::PointCloud<Point>    PointCloud;
typedef pcl::PointXYZRGB          PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

class PcdPublisher {

  // ROS properties
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publisher to send out the point cloud
  ros::Publisher point_cloud_pub_;

  // Timer for point cloud publication
  ros::WallTimer timer_;

  // PCD data
  std::string pcd_files_dir_;
  std::vector<std::string> pcd_files_;
  unsigned int pcd_counter_;

public:

  /**
   * Class constructor
   */
  PcdPublisher(std::string pcd_files_dir)
  {
    int len;
    double timer_period;

    // Node handlers
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");

    // Read parameters
    nh_private_.param("timer_period", timer_period, 0.1);

    // Init variables
    pcd_files_dir_ = pcd_files_dir;
    pcd_files_ = std::vector<std::string>();
    pcd_counter_ = 0;

    // Check if directory name finishes with "/"
    len = pcd_files_dir_.length();
    if (strcmp("/", &(pcd_files_dir_[len - 1])) != 0)
    {
      pcd_files_dir_ = pcd_files_dir_ + "/";
    }

    // List all pcd files into the directory
    listPcdFiles(pcd_files_dir_, pcd_files_);

    // Declare the point cloud topic
    point_cloud_pub_ = nh_private_.advertise<PointCloudRGB>("output", 1);

    // Init the timer
    timer_ = nh.createWallTimer(ros::WallDuration(timer_period), 
                            boost::bind(&PcdPublisher::readAndPublish,
                            this));
    ROS_INFO_STREAM("[PcdPublisher:] Timer started!");
  }

protected:

  /**
   * List PCD files into directory
   */
  int listPcdFiles(std::string dir, std::vector<std::string> &files)
  {
    int len;
    DIR *dp;
    struct dirent *dirp;

    // Check if directory exists
    if((dp  = opendir(dir.c_str())) == NULL)
    {
      std::cout << "Error(" << errno << ") opening " << dir << std::endl;
      return errno;
    }

    // List the .pcd files
    while ((dirp = readdir(dp)) != NULL)
    {
      len = strlen(dirp->d_name);
      if (len >= 4)
      {
        if (strcmp(".pcd", &(dirp->d_name[len - 4])) == 0)
        {
          files.push_back(dirp->d_name);
        }
      }
    }
    closedir(dp);

    // Sort the files alphabetically
    std::sort(files.begin(), files.end());
    return 0;
  }

  /**
   * Read and publish every point cloud
   */
  void readAndPublish()
  {
    // Check if all pcd files have been published
    if (pcd_counter_ < pcd_files_.size())
    {   
      PointCloud::Ptr cloud(new PointCloud);

      if (pcl::io::loadPCDFile<Point>(pcd_files_dir_ + pcd_files_[pcd_counter_], *cloud) == -1)
      {
        ROS_ERROR_STREAM("Couldn't read file " << pcd_files_dir_ + pcd_files_[pcd_counter_]);
        return;
      }

      // Convert the point cloud to RGB
      PointCloudRGB::Ptr cloud_rgb(new PointCloudRGB);
      pcl::copyPointCloud(*cloud, *cloud_rgb);

      // Publish the point cloud
      point_cloud_pub_.publish(cloud_rgb);
      
      ROS_INFO_STREAM("[PcdPublisher:] Published " << cloud->width * cloud->height 
        << " data points from " << pcd_files_dir_ + pcd_files_[pcd_counter_]);

      pcd_counter_++;
    }
    else
    {
      // All files processed
      timer_.stop();
      ROS_INFO_STREAM("[PcdPublisher:] All PCD files processed. Timer stoped!");
    }
  }
};

/**
 * Main entry point of the code
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcd_publisher");

  // First argument is the directory where pcd files are.
  std::string pcd_files_dir = argc > 1 ? argv[1] : "data";

  // Init the node
  PcdPublisher node(pcd_files_dir);
  ros::spin();
  return 0;
}

