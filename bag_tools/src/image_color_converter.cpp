#include "bag_tools/image_color_converter.h"


sensor_msgs::Image::Ptr bag_tools::toColor(const sensor_msgs::Image::ConstPtr& raw_img,
                                           const std::string& target_encoding)
{
    // Sanity checks.
    if (!raw_img)
        return nullptr;
    if (raw_img->encoding == target_encoding)
        return boost::make_shared<sensor_msgs::Image>(*raw_img);

    try
    {
        cv_bridge::CvImagePtr cv_converted = cv_bridge::cvtColor(cv_bridge::toCvCopy(raw_img), target_encoding);
        return cv_converted->toImageMsg();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[ImageColorConverter] cv_bridge conversion error: " << e.what());
        return nullptr;
    }
    
}