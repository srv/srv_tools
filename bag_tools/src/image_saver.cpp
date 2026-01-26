#include "bag_tools/image_saver.h"

ImageSaver::ImageSaver(const std::string& save_dir, 
                       const std::string& filetype, 
                       const std::string& prefix,
                       const int decimation):
                       num_saved_{0}, calib_saved_{false}, 
                       prefix_{prefix}, filetype_{filetype}, 
                       save_dir_{save_dir}            
{
    if (!std::filesystem::exists(save_dir_))
        std::filesystem::create_directories(save_dir_);

    scaler_.setDecimation(decimation, decimation);
}

void ImageSaver::save(const sensor_msgs::Image::ConstPtr& raw_img, 
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

    // Save image.
    std::string filename = (save_dir_ / (prefix_ + std::to_string(img_preproc->header.stamp.toNSec()) + "." + filetype_)).string();
    if (!cv::imwrite(filename, img_proc.rect_color))
    {
        ROS_ERROR_STREAM("ERROR Saving image: " << filename);
    }
    else
    {
        ROS_DEBUG_STREAM("Saved " << filename);
        num_saved_++;
    }
}