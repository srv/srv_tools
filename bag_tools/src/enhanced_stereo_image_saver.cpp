#include "bag_tools/enhanced_stereo_image_saver.h"


EnhancedStereoImageSaver::EnhancedStereoImageSaver(const std::string& save_dir, 
                                                   const std::string& filetype, 
                                                   const std::string& l_cam_name,
                                                   const std::string& r_cam_name,
                                                   const bool apply_clahe,
                                                   const bool apply_dehaze,
                                                   const int decimation,
                                                   const double clip_limit):
                                                   l_saver_(save_dir, filetype, ((!l_cam_name.empty() && l_cam_name[0] == '/') ? l_cam_name.substr(1) : l_cam_name) + "_", apply_clahe, apply_dehaze, decimation, clip_limit),
                                                   r_saver_(save_dir, filetype, ((!r_cam_name.empty() && r_cam_name[0] == '/') ? r_cam_name.substr(1) : r_cam_name) + "_", apply_clahe, apply_dehaze, decimation, clip_limit) {}

void EnhancedStereoImageSaver::save(const sensor_msgs::Image::ConstPtr& l_img, 
                                    const sensor_msgs::Image::ConstPtr& r_img, 
                                    const sensor_msgs::CameraInfo::ConstPtr& l_info,
                                    const sensor_msgs::CameraInfo::ConstPtr& r_info)
{
    l_saver_.save(l_img, l_info);
    r_saver_.save(r_img, r_info);
}