#include <ros/ros.h>

#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>

class PointCloudToWebgl {

// ROS properties
ros::NodeHandle nh_;
ros::NodeHandle nh_private_;

std::string pcd_filename_;
double max_ascii_file_size_;
int pcd_type_;

public:

  /**
   * Class constructor
   */
  PointCloudToWebgl() : nh_private_("~")
  {
    // Read the parameters from the parameter server (set defaults)
    nh_private_.param("pcd_filename", pcd_filename_, std::string("pointcloud_file.pcd"));
    nh_private_.param("max_ascii_file_size", max_ascii_file_size_, 4.0);  // In MBytes
    nh_private_.param("pcd_type", pcd_type_, 0);  // 0 -> XYZ | 1 -> XYZRGB

    ROS_INFO_STREAM("[PointCloudToWebgl:] Opening file " << pcd_filename_);

    // Read point cloud
    if (pcd_type_ == 0)
    {
      // NO RGB
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_filename_, *cloud_ptr) == -1) //* load the file
      {
        ROS_ERROR_STREAM("[PointCloudToWebgl:] Couldn't read file " << pcd_filename_);
      }
      else
      {
        // Convert the cloud
        pcl::PointCloud<pcl::PointXYZ> cloud = *cloud_ptr;
        int file_point_size = 27;
        int max_bytes = (int)round(max_ascii_file_size_ * 1024 * 1024);
        int desired_points = max_bytes / file_point_size;
        double voxel_size = 0.001;
        double offset = 0.0002;
        while ((int)cloud.size() > desired_points)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled = filter(cloud.makeShared(), voxel_size);
          cloud = *cloud_downsampled;
          voxel_size = voxel_size + offset;
        }

        // Compute the cloud centroid
        Eigen::Vector4f centroid; 
        pcl::compute3DCentroid(cloud, centroid);

        // Save int file
        int lastindex = pcd_filename_.find_last_of("."); 
        std::string filename = pcd_filename_.substr(0, lastindex); 
        filename = filename + ".txt";
        ROS_INFO_STREAM("[PointCloudToWebgl:] Saving webgl file to " << filename);
        std::fstream f_webgl(filename.c_str(), std::ios::out);
        for (unsigned int i=0; i<cloud.size(); i++)
        {
          f_webgl << cloud[i].x - centroid[0] << "," << 
                     cloud[i].y - centroid[1] << "," << 
                     cloud[i].z - centroid[2] << "," << 
                     (int)(224) << "," << 
                     (int)(224) << "," << 
                     (int)(224) << std::endl;
        }
        f_webgl.close();
        ROS_INFO("[PointCloudToWebgl:] Saved!");
      }
    }
    else
    {
      // RGB
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_filename_, *cloud_ptr) == -1) //* load the file
      {
        ROS_ERROR_STREAM("[PointCloudToWebgl:] Couldn't read file " << pcd_filename_);
      }
      else
      {
        // Convert the cloud
        pcl::PointCloud<pcl::PointXYZRGB> cloud = *cloud_ptr;
        int file_point_size = 37;
        int max_bytes = (int)round(max_ascii_file_size_ * 1024 * 1024);
        int desired_points = max_bytes / file_point_size;
        double voxel_size = 0.001;
        double offset = 0.0002;
        while ((int)cloud.size() > desired_points)
        {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled = filter(cloud.makeShared(), voxel_size);
          cloud = *cloud_downsampled;
          voxel_size = voxel_size + offset;
        }

        // Compute the cloud centroid
        Eigen::Vector4f centroid; 
        pcl::compute3DCentroid(cloud, centroid);

        // Save int file
        int lastindex = pcd_filename_.find_last_of("."); 
        std::string filename = pcd_filename_.substr(0, lastindex); 
        filename = filename + ".txt";
        ROS_INFO_STREAM("[PointCloudToWebgl:] Saving webgl file to " << filename);
        std::fstream f_webgl(filename.c_str(), std::ios::out);
        for (unsigned int i=0; i<cloud.size(); i++)
        {
          f_webgl << cloud[i].x - centroid[0] << "," << 
                     cloud[i].y - centroid[1] << "," << 
                     cloud[i].z - centroid[2] << "," << 
                     (int)cloud[i].r << "," << 
                     (int)cloud[i].g << "," << 
                     (int)cloud[i].b << std::endl;
        }
        f_webgl.close();
        ROS_INFO("[PointCloudToWebgl:] Saved!");
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double voxel_size)
  {
    // Downsampling using voxel grid
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> grid_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    grid_.setLeafSize(voxel_size,
                      voxel_size,
                      voxel_size);
    grid_.setDownsampleAllData(true);
    grid_.setInputCloud(cloud);
    grid_.filter(*cloud_downsampled_ptr);
    return cloud_downsampled_ptr;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double voxel_size)
  {
    // Downsampling using voxel grid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> grid_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    grid_.setLeafSize(voxel_size,
                      voxel_size,
                      voxel_size);
    grid_.setDownsampleAllData(true);
    grid_.setInputCloud(cloud);
    grid_.filter(*cloud_downsampled_ptr);
    return cloud_downsampled_ptr;
  }

};

/**
 * Main entry point of the code
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_to_webgl");
  PointCloudToWebgl node;
  ros::spin();
  return 0;
}

