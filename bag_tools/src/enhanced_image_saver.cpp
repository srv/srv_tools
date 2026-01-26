#include "bag_tools/enhanced_image_saver.h"

EnhancedImageSaver::EnhancedImageSaver(const std::string& save_dir, 
                                       const std::string& filetype, 
                                       const std::string& prefix,
                                       const bool apply_clahe,
                                       const bool apply_dehaze,
                                       const int decimation,
                                       const int clip_limit = 2):
                                       num_saved_{0}, calib_saved_{false},
                                       apply_clahe_{apply_clahe}, apply_dehaze_{apply_dehaze},
                                       prefix_{prefix}, filetype_{filetype}, 
                                       save_dir_{save_dir}            
{
    if (!std::filesystem::exists(save_dir_))
        std::filesystem::create_directories(save_dir_);

    scaler_.setDecimation(decimation, decimation);

    claher_ = std::make_unique<Clahe>(clip_limit);
}

void EnhancedImageSaver::save(const sensor_msgs::Image::ConstPtr& raw_img, 
                              const sensor_msgs::CameraInfo::ConstPtr& raw_info)
{
    // Pre-process (Crop & Decimate).
    auto [img_preproc, info_preproc] = scaler_.process(raw_img, raw_info);

    // Sanity check.
    if (!img_preproc || !info_preproc)
        return;

    // Save calibration (only once).
    camera_model_.fromCameraInfo(info_preproc);
    if (!calib_saved_)
    {
        std::string calib_filename = (save_dir_ / ("calibration_" + prefix_ + "camera.yaml")).string();
        if (camera_calibration_parsers::writeCalibration(calib_filename, prefix_ + "camera", *info_preproc))
        {
            ROS_DEBUG_STREAM("Calibration saved to: " << calib_filename);
            calib_saved_ = true;
        }
        else
        {
            ROS_ERROR_STREAM("Could not save calibration to: " << calib_filename);
        }
    }

    // Rectification.
    image_proc::ImageSet img_proc;
    if (!processor_.process(img_preproc, camera_model_, img_proc, image_proc::Processor::RECT_COLOR))
    {
        ROS_ERROR_STREAM("ERROR Processing image (rectification failed).");
        return;
    }

    cv::Mat out_img = img_proc.rect_color.clone();
    if (apply_dehaze_)
        out_img = dehazer_.dehazeRGB(out_img);
    if (apply_clahe_)
        out_img = claher_->correctBGR(out_img);
    
    // Save image.
    std::string filename = (save_dir_ / (prefix_ + std::to_string(img_preproc->header.stamp.toNSec()) + "." + filetype_)).string();
    if (!cv::imwrite(filename, out_img))
    {
        ROS_ERROR_STREAM("ERROR Saving image: " << filename);
    }
    else
    {
        ROS_DEBUG_STREAM("Saved " << filename);
        num_saved_++;
    }
}