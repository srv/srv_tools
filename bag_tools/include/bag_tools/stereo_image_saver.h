#ifndef STEREO_IMAGE_SAVER_H
#define STEREO_IMAGE_SAVER_H


#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "bag_tools/image_saver.h"


/** \brief Wrapper class to handle stereo pairs (left and right).
 */
class StereoImageSaver
{
    public:

        /** \brief Constructor.
         *  \param[in] save_dir Output directory.
         *  \param[in] filetype Image extension.
         *  \param[in] l_cam_name Name of left camera (used for topic matching and filename prefix).
         *  \param[in] r_cam_name Name of right camera.
         *  \param[in] decimation Decimation factor.
         */
        StereoImageSaver(const std::string& save_dir, 
                         const std::string& filetype, 
                         const std::string& l_cam_name,
                         const std::string& r_cam_name,
                         const int decimation);

        /** \brief Main callback for synchronized stereo messages.
         */
        void save(const sensor_msgs::Image::ConstPtr& l_img, 
                  const sensor_msgs::Image::ConstPtr& r_img, 
                  const sensor_msgs::CameraInfo::ConstPtr& l_info,
                  const sensor_msgs::CameraInfo::ConstPtr& r_info);

        private:

            ImageSaver l_saver_, r_saver_;

};

#endif // STEREO_IMAGE_SAVER_H