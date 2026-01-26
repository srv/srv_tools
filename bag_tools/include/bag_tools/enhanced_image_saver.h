#ifndef ENHANCED_IMAGE_SAVER_H
#define ENHANCED_IMAGE_SAVER_H

#include <string>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_proc/processor.h>
#include <camera_calibration_parsers/parse.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "bag_tools/image_resolution_scaler.h"
#include "image_preprocessing/dehazer.h"
#include "image_preprocessing/clahe.h"


/** \brief Saves rectified color images from raw and camera info input. It can decimate the images.
 *         Manages the pipeline: Preprocess -> Save Calib -> Rectify -> Save Image.
 */
class EnhancedImageSaver
{

    public:

        /** \brief Constructor.
         *  \param[in] save_dir Directory path to save output files.
         *  \param[in] filetype Image extension (e.g., "jpg", "png").
         *  \param[in] prefix Filename prefix (usually camera name, e.g., "left_").
         *  \param[in] decimation Downsampling factor to apply.
         */
        EnhancedImageSaver(const std::string& save_dir, 
                           const std::string& filetype, 
                           const std::string& prefix,
                           const bool apply_clahe,
                           const bool apply_dehaze,
                           const int decimation,
                           const int clip_limit);

        /** \brief Callback to process and save a single frame.
         *  \param[in] raw_img Raw image message.
         *  \param[in] raw_info Camera info message.
         */
        void save(const sensor_msgs::Image::ConstPtr& raw_img, 
                  const sensor_msgs::CameraInfo::ConstPtr& raw_info);

    private:

        int num_saved_;

        bool calib_saved_;
        bool apply_clahe_;
        bool apply_dehaze_;

        std::string prefix_;
        std::string filetype_;

        ImageResolutionScaler scaler_;

        std::filesystem::path save_dir_;
        
        image_proc::Processor processor_;
        image_geometry::PinholeCameraModel camera_model_;

        std::unique_ptr<Clahe> claher_;

        Dehazer dehazer_;
};


#endif // ENHANCED_IMAGE_SAVER_H