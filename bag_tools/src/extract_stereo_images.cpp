#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_proc/processor.h>
#include <camera_calibration_parsers/parse.h>

#include <opencv2/highgui/highgui.hpp>

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    signalMessage(msg);
  }
};

/**
 * Saves rectified color images from raw and camera info input
 */
class ImageSaver
{
public:
  ImageSaver(const std::string& save_dir, const std::string& filetype, 
      const std::string& prefix) :
    save_dir_(save_dir), filetype_(filetype), prefix_(prefix), num_saved_(0)
  {}

  void save(const sensor_msgs::Image::ConstPtr &img, 
            const sensor_msgs::CameraInfo::ConstPtr &info)
  {
    camera_model_.fromCameraInfo(info);
    std::string calib_filename_ini = 
      boost::str(boost::format("%s/calibration_%scamera.ini") 
          % save_dir_ % prefix_ );
    std::string calib_filename_yaml = 
      boost::str(boost::format("%s/calibration_%scamera.yaml") 
          % save_dir_ % prefix_ );
    camera_calibration_parsers::writeCalibration(calib_filename_ini, prefix_ + "camera", *info);
    camera_calibration_parsers::writeCalibration(calib_filename_yaml, prefix_ + "camera", *info);
    image_proc::ImageSet output;
    if (!processor_.process(img, camera_model_, output, image_proc::Processor::RECT_COLOR))
    {
      std::cerr << "ERROR Processing image" << std::endl;
      return;
    }
    std::string filename = 
      boost::str(boost::format("%s/%s%lu.%s") 
          % save_dir_ 
          % prefix_ 
          % img->header.stamp.toNSec() 
          % filetype_);
    if (!cv::imwrite(filename, output.rect_color))
      std::cerr << "ERROR Saving " << filename << std::endl;
    else
    {
      std::cout << "INFO  Saved " << filename << std::endl;
      num_saved_++;
    }
  }

private:

  image_proc::Processor processor_;
  image_geometry::PinholeCameraModel camera_model_;
  std::string save_dir_;
  std::string filetype_;
  std::string prefix_;
  int num_saved_;

};

class StereoImageSaver
{
public:
  StereoImageSaver(const std::string& save_dir, const std::string& filetype) :
    l_saver_(save_dir, filetype, "left_"),
    r_saver_(save_dir, filetype, "right_")
  {}

  void save(const sensor_msgs::Image::ConstPtr &l_img, 
            const sensor_msgs::Image::ConstPtr &r_img, 
            const sensor_msgs::CameraInfo::ConstPtr &l_info,
            const sensor_msgs::CameraInfo::ConstPtr &r_info)
  {
    l_saver_.save(l_img, l_info);
    r_saver_.save(r_img, r_info);
  }

private:

  ImageSaver l_saver_, r_saver_;

};
 
class StereoBagProcessor
{

public:
  StereoBagProcessor(const std::string& stereo_base_topic) :
    stereo_base_topic_(stereo_base_topic),
    sync_(l_img_sub_, r_img_sub_, l_info_sub_, r_info_sub_, 25)
  {
  }

  template<class C>
  void registerCallback(const C& callback)
  {
    sync_.registerCallback(callback);
  }

  void processBag(const std::string &filename)
  {
    std::cout << "Starting processing " << filename << std::endl;

    // Image topics to load
    std::string l_cam = stereo_base_topic_ + "/left";
    std::string r_cam = stereo_base_topic_ + "/right";
    std::string l_cam_image = l_cam + "/image_raw";
    std::string r_cam_image = r_cam + "/image_raw";
    std::string l_cam_info = l_cam + "/camera_info";
    std::string r_cam_info = r_cam + "/camera_info";

    std::vector<std::string> topics;
    topics.push_back(l_cam_image);
    topics.push_back(r_cam_image);
    topics.push_back(l_cam_info);
    topics.push_back(r_cam_info);
    
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    // sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
    
    // Load all messages
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image))
      {
        sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
        if (l_img != NULL)
          l_img_sub_.newMessage(l_img);
      }
      
      if (m.getTopic() == r_cam_image || ("/" + m.getTopic() == r_cam_image))
      {
        sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
        if (r_img != NULL)
          r_img_sub_.newMessage(r_img);
      }
      
      if (m.getTopic() == l_cam_info || ("/" + m.getTopic() == l_cam_info))
      {
        sensor_msgs::CameraInfo::ConstPtr l_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (l_info != NULL)
          l_info_sub_.newMessage(l_info);
      }
      
      if (m.getTopic() == r_cam_info || ("/" + m.getTopic() == r_cam_info))
      {
        sensor_msgs::CameraInfo::ConstPtr r_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (r_info != NULL)
          r_info_sub_.newMessage(r_info);
      }
    }
    bag.close();
    std::cout << "Finished processing " << filename << std::endl;
  }

private:

  // Fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> l_img_sub_, r_img_sub_;
  BagSubscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;

  std::string stereo_base_topic_;
  
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_;

};


int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: " << argv[0] << " OUT_DIR FILETYPE STEREO_BASE_TOPIC BAGFILE [BAGFILE...]" << std::endl;
    std::cout << "  Example: " << argv[0] << " /tmp jpg /stereo_down bag1.bag bag2.bag" << std::endl;
    return 0;
  }

  std::string out_dir(argv[1]);
  std::string filetype(argv[2]);
  std::string base_topic(argv[3]);
  
  ros::Time::init();

  StereoImageSaver saver(out_dir, filetype);
  StereoBagProcessor processor(base_topic);
  processor.registerCallback(boost::bind(&StereoImageSaver::save, saver, _1, _2, _3, _4));

  for (int i = 4; i < argc; ++i)
    processor.processBag(argv[i]);

  return 0;
}

