#ifndef BAG_TOOLS_IMAGE_COLOR_CONVERTER_H
#define BAG_TOOLS_IMAGE_COLOR_CONVERTER_H


#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


namespace bag_tools
{

    /** \brief Converts a raw image to BGR8 (debayering if necessary).
     *  \param[in] raw_img Input image message.
     *  \param[in] target_encoding Desired output encoding (default: BGR8).
     *  \return Pointer to converted sensor_msgs::Image, or original image if no conversion needed.
     *          Returns nullptr on failure.
     */
    sensor_msgs::Image::Ptr toColor(const sensor_msgs::Image::ConstPtr& raw_img,
                                    const std::string& target_encoding = sensor_msgs::image_encodings::BGR8);

} // namespace bag_tools


#endif // BAG_TOOLS_IMAGE_COLOR_CONVERTER_H