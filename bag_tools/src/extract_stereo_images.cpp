/// Copyright (c) 2012,
/// Systems, Robotics and Vision Group
/// University of the Balearican Islands
/// All rights reserved.
/// 
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * Neither the name of Systems, Robotics and Vision Group, University of 
///       the Balearican Islands nor the names of its contributors may be used 
///       to endorse or promote products derived from this software without 
///       specific prior written permission.
/// 
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_proc/processor.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_calibration_parsers/parse.h>

#include <bag_tools/stereo_bag_processor.h>


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
class Preprocessor
{

  public:


    /** \brief Empty class constructor.
     */
    Preprocessor():
                 roi_x_{0}, roi_y_{0},
                 roi_width_{0}, roi_height_{0},
                 decimation_x_{1}, decimation_y_{1} {}


    /** \brief ROI class constructor.
     *  \param[in] roi_x X coordinate of the top-left corner of the ROI.
     *  \param[in] roi_y Y coordinate of the top-left corner of the ROI.
     *  \param[in] roi_width Width of the desired ROI.
     *  \param[in] roi_height Height of the desired ROI.
     */
    Preprocessor(const int roi_x,
                 const int roi_y,
                 const int roi_width,
                 const int roi_height):
                 roi_x_{roi_x}, roi_y_{roi_y},
                 roi_width_{roi_width}, roi_height_{roi_height},
                 decimation_x_{1}, decimation_y_{1} {}


    /** \brief Decimation class constructor.
     *  \param[in] decimation_x Decimation factor in X (e.g., 2 means half resolution).
     *  \param[in] decimation_y Decimation factor in Y.
     */
    Preprocessor(const int decimation_x,
                 const int decimation_y):
                 roi_x_{0}, roi_y_{0},
                 roi_width_{0}, roi_height_{0},
                 decimation_x_{decimation_x}, decimation_y_{decimation_y} {}


    /** \brief Full class constructor.
     *  \param[in] roi_x X coordinate of the top-left corner of the ROI.
     *  \param[in] roi_y Y coordinate of the top-left corner of the ROI.
     *  \param[in] roi_width Width of the desired ROI.
     *  \param[in] roi_height Height of the desired ROI.
     *  \param[in] decimation_x Decimation factor in X (e.g., 2 means half resolution).
     *  \param[in] decimation_y Decimation factor in Y.
     */
    Preprocessor(const int roi_x,
                 const int roi_y,
                 const int roi_width,
                 const int roi_height,
                 const int decimation_x,
                 const int decimation_y):
                 roi_x_{roi_x}, roi_y_{roi_y},
                 roi_width_{roi_width}, roi_height_{roi_height},
                 decimation_x_{decimation_x}, decimation_y_{decimation_y} {}

    
    /** \brief Sets the decimation (downsampling) factor.
     *  \param[in] x Decimation factor in X axis (>= 1).
     *  \param[in] y Decimation factor in Y axis (>= 1).
     */
    void setDecimation(const int x, 
                       const int y)
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
    void setROI(int x, 
                int y, 
                int width, 
                int height)
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


  private:

    int roi_x_, roi_y_, roi_width_, roi_height_;
    int decimation_x_, decimation_y_; 
};


/** \brief Saves rectified color images from raw and camera info input. It can decimate the images.
 *         Manages the pipeline: Preprocess -> Save Calib -> Rectify -> Save Image.
 */
class ImageSaver
{

  public:


    /** \brief Constructor.
     *  \param[in] save_dir Directory path to save output files.
     *  \param[in] filetype Image extension (e.g., "jpg", "png").
     *  \param[in] prefix Filename prefix (usually camera name, e.g., "left_").
     *  \param[in] decimation Downsampling factor to apply.
     */
    ImageSaver(const std::string& save_dir, 
               const std::string& filetype, 
               const std::string& prefix,
               const int decimation):
               num_saved_{0}, calib_saved_{false}, 
               prefix_{prefix}, filetype_{filetype}, 
               save_dir_{save_dir}            
    {
      if (!std::filesystem::exists(save_dir_))
        std::filesystem::create_directories(save_dir_);

      preprocessor_.setDecimation(decimation, decimation);
    }


    /** \brief Callback to process and save a single frame.
     *  \param[in] raw_img Raw image message.
     *  \param[in] raw_info Camera info message.
     */
    void save(const sensor_msgs::Image::ConstPtr& raw_img, 
              const sensor_msgs::CameraInfo::ConstPtr& raw_info)
    {
      // Pre-process (Crop & Decimate).
      auto [img_preproc, info_preproc] = preprocessor_.process(raw_img, raw_info);

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


  private:

    int num_saved_;

    bool calib_saved_;

    std::string prefix_;
    std::string filetype_;

    Preprocessor preprocessor_;

    std::filesystem::path save_dir_;
    
    image_proc::Processor processor_;
    image_geometry::PinholeCameraModel camera_model_;


};


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
                     const int decimation):
                    l_saver_(save_dir, filetype, ((!l_cam_name.empty() && l_cam_name[0] == '/') ? l_cam_name.substr(1) : l_cam_name) + "_", decimation),
                    r_saver_(save_dir, filetype, ((!r_cam_name.empty() && r_cam_name[0] == '/') ? r_cam_name.substr(1) : r_cam_name) + "_", decimation) {}


    /** \brief Main callback for synchronized stereo messages.
     */
    void save(const sensor_msgs::Image::ConstPtr& l_img, 
              const sensor_msgs::Image::ConstPtr& r_img, 
              const sensor_msgs::CameraInfo::ConstPtr& l_info,
              const sensor_msgs::CameraInfo::ConstPtr& r_info)
    {
      l_saver_.save(l_img, l_info);
      r_saver_.save(r_img, r_info);
    }


  private:


    ImageSaver l_saver_, r_saver_;


};


int main(int argc, char** argv)
{
  if (argc < 8)
  {
    std::cout << "Saves rectified color images from raw and camera info input. It is possible to decimate images if the `decimation` parameter is greater than one." << std::endl;
    std::cout << "Usage: " << argv[0] << " OUT_DIR FILETYPE STEREO_BASE_TOPIC LEFT_CAMERA_NAME RIGHT_CAMERA_NAME DECIMATION BAGFILE [BAGFILE...]" << std::endl;
    std::cout << "  Example: " << argv[0] << " /tmp jpg /stereo_down /left /right 1 bag1.bag bag2.bag" << std::endl;
    return 0;
  }

  // Parsing.
  char *p;
  std::string out_dir(argv[1]);
  std::string filetype(argv[2]);
  std::string base_topic(argv[3]);
  std::string l_cam_name(argv[4]);
  std::string r_cam_name(argv[5]);
  long decimation_long = std::strtol(argv[6], &p, 10);
  int decimation = static_cast<int>(decimation_long);
  
  ros::Time::init();

  StereoImageSaver saver(out_dir, filetype, l_cam_name, r_cam_name, decimation);
  bag_tools::StereoBagProcessor processor(base_topic, l_cam_name, r_cam_name);
  processor.registerCallback(boost::bind(&StereoImageSaver::save, &saver, _1, _2, _3, _4));

  for (int i = 7; i < argc; ++i)
    processor.processBag(argv[i]);

  return 0;
}