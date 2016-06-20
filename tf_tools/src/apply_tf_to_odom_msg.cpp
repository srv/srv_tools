#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;

class ApplyTF2Odom
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;

  tf::Transform trans_;

public:
  ApplyTF2Odom() : nh_private_("~")
  {
    // Read parameters
    double x, y, z, qx, qy, qz, qw;
    nh_private_.param("x", x, 0.0);
    nh_private_.param("y", y, 0.0);
    nh_private_.param("z", z, 0.0);
    nh_private_.param("qx", qx, 0.0);
    nh_private_.param("qy", qy, 0.0);
    nh_private_.param("qz", qz, 0.0);
    nh_private_.param("qw", qw, 1.0);

    // Build tf
    tf::Vector3 t(x, y, z);
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Transform tmp(q, t);
    trans_ = tmp;

    // Messages
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odometry_in", 1, &ApplyTF2Odom::callback, this);
    odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odometry_out", 1);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    // Get the data
    double x = odom_msg->pose.pose.position.x;
    double y = odom_msg->pose.pose.position.y;
    double z = odom_msg->pose.pose.position.z;

    double qx = odom_msg->pose.pose.orientation.x;
    double qy = odom_msg->pose.pose.orientation.y;
    double qz = odom_msg->pose.pose.orientation.z;
    double qw = odom_msg->pose.pose.orientation.w;

    // Build the odometry pose
    tf::Vector3 t(x, y, z);
    tf::Quaternion q(qx, qy, qz, 1.0);
    tf::Transform pose(q, t);

    // Transform
    tf::Transform new_pose = trans_ * pose;

    // Publish
    nav_msgs::Odometry new_odom_msg = *odom_msg;
    tf::poseTFToMsg(new_pose, new_odom_msg.pose.pose);
    odom_pub_.publish(new_odom_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apply_tf_to_odom_msg");
  ApplyTF2Odom node;
  ros::spin();
  return 0;
}

