#ifndef HHCM_BT_CPP_LIBS_PORT_ROS_TIME_CONVERSIONS
#define HHCM_BT_CPP_LIBS_PORT_ROS_TIME_CONVERSIONS

#include <behaviortree_cpp/basic_types.h>
#include <ros/duration.h>

// Template specialization to converts a string to geometry_msgs::transformStamped.
namespace BT
{
    //string must be in seconds
    template <> inline ros::Duration convertFromString(StringView str)
    {
        return ros::Duration(convertFromString<double>(str));
    }    

    template <>
    std::string toStr(const ros::Duration& duration) 
    {
        double sec = duration.toSec();

        //std::string old_locale = setlocale(LC_NUMERIC, nullptr);
        //setlocale(LC_NUMERIC, "C");
        std::string sec_str = std::to_string(sec);
        //setlocale(LC_NUMERIC, old_locale.c_str());

        return sec_str;
    }
    
} // end namespace BT

#endif // HHCM_BT_CPP_LIBS_PORT_ROS_TIME_CONVERSIONS
