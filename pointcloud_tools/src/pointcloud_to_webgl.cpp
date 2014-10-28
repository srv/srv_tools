#include <ros/ros.h>

#include <boost/lexical_cast.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

class PointCloudToWebgl {

public:

  /**
   * Class constructor
   */
  PointCloudToWebgl(const std::string& input_cloud, const int& cloud_format,
      const std::string& output_cloud) :
    input_cloud_(input_cloud), cloud_format_(cloud_format), output_cloud_(output_cloud)
  {}

  /**
   * Convert the cloud
   */
  void convert()
  {
    ROS_INFO_STREAM("[PointCloudToWebgl:] Opening file " << input_cloud_);

    // Init the cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Read point cloud
    if (cloud_format_ == 0)
    {
      // NO RGB
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_cloud_, *cloud_xyz) == -1) //* load the file
      {
        ROS_ERROR_STREAM("[PointCloudToWebgl:] Couldn't read file " << input_cloud_);
        return;
      }
      copyPointCloud(*cloud_xyz, cloud);
    }
    else if (cloud_format_ == 1)
    {
      // RGB
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_cloud_, *cloud_xyzrgb) == -1) //* load the file
      {
        ROS_ERROR_STREAM("[PointCloudToWebgl:] Couldn't read file " << input_cloud_);
        return;
      }
      copyPointCloud(*cloud_xyzrgb, cloud);
    }

    // Compute the cloud centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud, centroid);

    // Save int file
    ROS_INFO_STREAM("[PointCloudToWebgl:] Saving webgl file to " << output_cloud_);
    std::fstream f_webgl(output_cloud_.c_str(), std::ios::out);
    for (unsigned int i=0; i<cloud.size(); i++)
    {
      int r = 224;
      int g = 224;
      int b = 224;
      if (cloud_format_ == 1)
      {
        r = (int)cloud[i].r;
        g = (int)cloud[i].g;
        b = (int)cloud[i].b;
      }
      f_webgl << cloud[i].x - centroid[0] << "," <<
                 cloud[i].y - centroid[1] << "," <<
                 cloud[i].z - centroid[2] << "," <<
                 r << "," <<
                 g << "," <<
                 b << std::endl;

    }
    f_webgl.close();
    ROS_INFO("[PointCloudToWebgl:] Saved!");
  }

  private:
  std::string input_cloud_;
  std::string output_cloud_;
  int cloud_format_;
};

/**
 * Main entry point of the code
 */
int main(int argc, char **argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: " << argv[0] << " INPUT_PCD FORMAT OUTPUT_CSV" << std::endl;
    std::cout << "  Example: " << argv[0] << " input_cloud.pcd 0 output_cloud.csv" << std::endl;
    return 0;
  }

  // Read inputs
  std::string input_cloud(argv[1]);
  std::string output_cloud(argv[3]);
  int cloud_format = boost::lexical_cast<int>(argv[2]);

  // Convert
  PointCloudToWebgl converter(input_cloud, cloud_format, output_cloud);
  converter.convert();

  return 0;
}

