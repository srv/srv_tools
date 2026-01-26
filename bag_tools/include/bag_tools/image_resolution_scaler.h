#ifndef IMAGE_RESOLUTION_SCALER_H
#define IMAGE_RESOLUTION_SCALER_H


#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


/** \brief Handles cropping and decimation of Raw Images and adjusts CameraInfo metadata.
 * 
 *  This class performs two main operations:
 *  1. Cropping (ROI): Cuts the image and adjusts the ROI offset in CameraInfo.
 *  2. Decimation (Binning): Resizes the image and updates binning fields in CameraInfo.
 * 
 *  \note It does NOT manually scale K/P matrices. It relies on the standard ROS 
 *        mechanism where `image_geometry` uses the `binning_x/y` and `roi` fields 
 *        combined with the original K/P to calculate rectification maps correctly.
 */
class ImageResolutionScaler
{

    public:

        /** \brief Empty class constructor.
         */
        ImageResolutionScaler();

        /** \brief ROI class constructor.
         *  \param[in] roi_x X coordinate of the top-left corner of the ROI.
         *  \param[in] roi_y Y coordinate of the top-left corner of the ROI.
         *  \param[in] roi_width Width of the desired ROI.
         *  \param[in] roi_height Height of the desired ROI.
         */
        ImageResolutionScaler(const int roi_x,
                              const int roi_y,
                              const int roi_width,
                              const int roi_height);

        /** \brief Decimation class constructor.
         *  \param[in] decimation_x Decimation factor in X (e.g., 2 means half resolution).
         *  \param[in] decimation_y Decimation factor in Y.
         */
        ImageResolutionScaler(const int decimation_x,
                              const int decimation_y);

        /** \brief Full class constructor.
         *  \param[in] roi_x X coordinate of the top-left corner of the ROI.
         *  \param[in] roi_y Y coordinate of the top-left corner of the ROI.
         *  \param[in] roi_width Width of the desired ROI.
         *  \param[in] roi_height Height of the desired ROI.
         *  \param[in] decimation_x Decimation factor in X (e.g., 2 means half resolution).
         *  \param[in] decimation_y Decimation factor in Y.
         */
        ImageResolutionScaler(const int roi_x,
                              const int roi_y,
                              const int roi_width,
                              const int roi_height,
                              const int decimation_x,
                              const int decimation_y);

        /** \brief Sets the decimation (downsampling) factor.
         *  \param[in] x Decimation factor in X axis (>= 1).
         *  \param[in] y Decimation factor in Y axis (>= 1).
         */
        inline void setDecimation(const int x, const int y)
        {
            decimation_x_ = std::max(1, x);
            decimation_y_ = std::max(1, y);
        }

        /** \brief Sets the Region of Interest (Crop).
         *  \param[in] x Top-left X coordinate.
         *  \param[in] y Top-left Y coordinate.
         *  \param[in] width ROI width.
         *  \param[in] height ROI height.
         */
        inline void setROI(const int x, const int y, const int width, const int height)
        {
            roi_x_ = std::max(0, x);
            roi_y_ = std::max(0, y);
            roi_width_ = std::max(0, width);
            roi_height_ = std::max(0, height);
        }

        /** \brief Processes a raw image and its camera info.
         *  \param[in] raw_img Input raw image (can be Bayer or Color).
         *  \param[in] raw_info Input camera info corresponding to the raw image.
         *  @return A pair containing the processed Image and the modified CameraInfo. 
         *          Returns {nullptr, nullptr} on failure.
         */
        std::pair<sensor_msgs::Image::Ptr, sensor_msgs::CameraInfo::Ptr> process(const sensor_msgs::Image::ConstPtr& raw_img,
                                                                                 const sensor_msgs::CameraInfo::ConstPtr& raw_info);

    
        private:

            int roi_x_, roi_y_, roi_width_, roi_height_;
            int decimation_x_, decimation_y_; 

};


#endif // IMAGE_RESOLUTION_SCALER_H