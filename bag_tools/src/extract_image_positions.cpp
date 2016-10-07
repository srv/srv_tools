#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <auv_msgs/NavSts.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

 // Topic sync
  typedef message_filters::sync_policies::ApproximateTime<auv_msgs::NavSts,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

class ImagePositions {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber cam_sub_;
  image_transport::CameraSubscriber sub_;
  tf::TransformListener tf_listener_;
  std::string parent_frame_id_;
  std::string child_frame_id_;
  std::string output_file_;
  std::string output_folder_;
  std::ofstream file_;
  int count_;

public:
  ImagePositions(const std::vector<std::string>& frame_ids)
    : it_(nh_), count_(0){
    nh_.getParam("parent_frame_id", parent_frame_id_);
    nh_.getParam("child_frame_id", child_frame_id_);
    nh_.getParam("output_file", output_file_);
    nh_.getParam("output_folder", output_folder_);
    std::string fn(output_folder_ + "/" + output_file_);
    file_.open(fn.c_str());
    file_ << "name, lat, lon, depth, altitude, roll, pitch, yaw\n";
    std::string camera_topic = nh_.resolveName("camera");
    std::string nav_sts_topic = nh_.resolveName("nav_sts");

    image_transport::SubscriberFilter image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    message_filters::Subscriber<auv_msgs::NavSts> nav_sts_sub;

    // Message sync
    boost::shared_ptr<Sync> sync;
    nav_sts_sub.subscribe(nh_, nav_sts_topic, 20);
    image_sub  .subscribe(it_, camera_topic + "/image_rect_color", 5);
    info_sub   .subscribe(nh_, camera_topic + "/camera_info", 5);
    sync.reset(new Sync(SyncPolicy(10), nav_sts_sub, image_sub, info_sub) );
    sync->registerCallback(boost::bind(&ImagePositions::msgsCallback, this, _1, _2, _3));
  }

  ~ImagePositions() {
    file_.close();
  }

  void msgsCallback(const auv_msgs::NavStsConstPtr& nav_sts_msg,
                    const sensor_msgs::ImageConstPtr& image_msg,
                    const sensor_msgs::CameraInfoConstPtr& info_msg) {
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[ImagePositions] Failed to convert image");
      return;
    }

    double lat      = nav_sts_msg->position.north;
    double lon      = nav_sts_msg->position.east;
    double depth    = nav_sts_msg->position.depth;
    double altitude = nav_sts_msg->altitude;
    double roll     = nav_sts_msg->orientation.roll;
    double pitch    = nav_sts_msg->orientation.pitch;
    double yaw      = nav_sts_msg->orientation.yaw;

    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << count_;
    std::string name = ss.str() + ".png";

    ROS_INFO_STREAM("[ImagePositions]: Saving image " << name);
    cv::imwrite(output_folder_ + "/" + name, image);

    file_ << name      << ", "
          << lat       << ", "
          << lon       << ", "
          << depth     << ", "
          << altitude  << ", "
          << roll      << ", "
          << pitch     << ", "
          << yaw       << "\n";
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_positions");
  ImagePositions ip();
  ros::spin();
}