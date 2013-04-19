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

class PcdPublisher {

  // ROS properties
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publisher to send out the point cloud
  ros::Publisher point_cloud_pub_;

public:

  /**
   * Class constructor
   */
  PcdPublisher(std::string pcd_files_dir) : nh_private_("~")
  {
    // Declare the point cloud topic
    point_cloud_pub_ = nh_private_.advertise<PointCloud>("points2", 1);

    readAndPublish(pcd_files_dir);
  }

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
  void readAndPublish(std::string pcd_files_dir)
  {
    int len;
    std::vector<std::string> files = std::vector<std::string>();

    // Check if directory name finishes with "/"
    len = pcd_files_dir.length();
    if (strcmp("/", &(pcd_files_dir[len - 1])) != 0)
    {
      pcd_files_dir = pcd_files_dir + "/";
    }

    // List 
    listPcdFiles(pcd_files_dir,files);

    // Read and publish every PCD
    for (unsigned int i = 0;i < files.size();i++)
    {
      PointCloud::Ptr cloud (new PointCloud);

      if (pcl::io::loadPCDFile<Point>(pcd_files_dir + files[i], *cloud) == -1)
      {
        ROS_ERROR_STREAM("Couldn't read file " << pcd_files_dir + files[i]);
        return;
      }

      // Publish the point cloud
      point_cloud_pub_.publish(cloud);
      
      ROS_INFO_STREAM("Loaded and published " << cloud->width * cloud->height 
        << " data points from " << pcd_files_dir + files[i]);
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

