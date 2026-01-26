#include "bag_tools/image_resolution_scaler.h"


ImageResolutionScaler::ImageResolutionScaler(): roi_x_{0}, roi_y_{0},
                                                roi_width_{0}, roi_height_{0},
                                                decimation_x_{1}, decimation_y_{1} {}

ImageResolutionScaler::ImageResolutionScaler(const int roi_x,
                                             const int roi_y,
                                             const int roi_width,
                                             const int roi_height):
                                             roi_x_{roi_x}, roi_y_{roi_y},
                                             roi_width_{roi_width}, roi_height_{roi_height},
                                             decimation_x_{1}, decimation_y_{1} {}

ImageResolutionScaler::ImageResolutionScaler(const int decimation_x,
                                             const int decimation_y):
                                             roi_x_{0}, roi_y_{0},
                                             roi_width_{0}, roi_height_{0},
                                             decimation_x_{decimation_x}, decimation_y_{decimation_y} {}

ImageResolutionScaler::ImageResolutionScaler(const int roi_x,
                                             const int roi_y,
                                             const int roi_width,
                                             const int roi_height,
                                             const int decimation_x,
                                             const int decimation_y):
                                             roi_x_{roi_x}, roi_y_{roi_y},
                                             roi_width_{roi_width}, roi_height_{roi_height},
                                             decimation_x_{decimation_x}, decimation_y_{decimation_y} {}

std::pair<sensor_msgs::Image::Ptr, sensor_msgs::CameraInfo::Ptr> ImageResolutionScaler::process(const sensor_msgs::Image::ConstPtr& raw_img,
                                                                                                const sensor_msgs::CameraInfo::ConstPtr& raw_info)
{
    // Copy camera info to modify metadata.
    sensor_msgs::CameraInfo::Ptr info_out(new sensor_msgs::CameraInfo(*raw_info));

    // From ROS to OpenCV.
    cv_bridge::CvImagePtr cv_img_ptr;
    try
    {
    cv_img_ptr = cv_bridge::toCvCopy(raw_img, raw_img->encoding);
    }
    catch (const cv_bridge::Exception& e)
    {
    ROS_ERROR_STREAM("[Caster:] cv_bridge exception: " << e.what());
    return {nullptr, nullptr};
    }

    // ---- CROP ----
    // Calculate the safe intersection.
    cv::Rect image_rect(0, 0, cv_img_ptr->image.cols, cv_img_ptr->image.rows);
    cv::Rect target_roi(roi_x_, roi_y_, roi_width_, roi_height_);

    // If the configured ROI is 0 or invalid, assume full image.
    bool valid_roi = (roi_width_ > 0 && roi_height_ > 0);
    cv::Rect final_roi = valid_roi ? (target_roi & image_rect) : image_rect;

    // Apply Crop to the image.
    if (final_roi != image_rect) 
    {
    cv_img_ptr->image = cv_img_ptr->image(final_roi);
    
    // Update ROI metadata in CameraInfo.
    // Note: ROS accumulates the offset if it already had a previous offset.
    info_out->roi.x_offset += final_roi.x;
    info_out->roi.y_offset += final_roi.y;
    info_out->roi.width = final_roi.width;
    info_out->roi.height = final_roi.height;
    info_out->roi.do_rectify = true; // Important for image_geometry to take into account.
    }

    // ---- DECIMATION (BINNING) ----
    if (decimation_x_ > 1 || decimation_y_ > 1) 
    {
    double scale_x = 1.0 / decimation_x_;
    double scale_y = 1.0 / decimation_y_;

    // Resize with Nearest Neighbour to avoid corrupting the Bayer pattern.
    cv::resize(cv_img_ptr->image, cv_img_ptr->image, cv::Size(), scale_x, scale_y, cv::INTER_NEAREST);

    // Update intrinisic parameters.
    info_out->K[0] *= scale_x;                                  // fx
    info_out->K[2] = (info_out->K[2] - final_roi.x) * scale_x;  // cx
    info_out->K[4] *= scale_y;                                  // fy
    info_out->K[5] = (info_out->K[5] - final_roi.y) * scale_y;  // cy

    // Update extrinsic parameters.
    info_out->P[0] *= scale_x;                                  // fx'
    info_out->P[2] = (info_out->P[2] - final_roi.x) * scale_x;  // cx'
    info_out->P[3] *= scale_x;                                  // Tx
    info_out->P[5] *= scale_y;                                  // fy'
    info_out->P[6] = (info_out->P[6] - final_roi.y) * scale_y;  // cy'
    info_out->P[7] *= scale_y;                                  // Ty
    
    // Update final size in the message.
    info_out->width = cv_img_ptr->image.cols;
    info_out->height = cv_img_ptr->image.rows;
    }

    return {cv_img_ptr->toImageMsg(), info_out};

}