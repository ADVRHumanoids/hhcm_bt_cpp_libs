#ifndef HHCM_BT_CPP_LIBS_PORT_GEOMETRY_MSGS_CONVERSIONS
#define HHCM_BT_CPP_LIBS_PORT_GEOMETRY_MSGS_CONVERSIONS

#include <behaviortree_cpp/basic_types.h> //for stringview
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

// Template specialization to converts a string to geometry_msgs::transformStamped.
namespace BT
{
    template <> inline geometry_msgs::Vector3 convertFromString(StringView str)
    {
        if (str.size() == 0) {
            throw RuntimeError("empty stringview");
        }
        
        auto parts = splitString(str, ',');
        if (parts.size() != 3)
        {
            std::string err_str = 
                "invalid input: expected this format: 'x, y, z', received '" + std::string(str) + "'";
            throw RuntimeError(err_str);
        }
        else{
            geometry_msgs::Vector3 output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            
            return output;
        }
    }    
    
    template <> inline geometry_msgs::Point convertFromString(StringView str)
    {
        if (str.size() == 0) {
            throw RuntimeError("empty stringview");
        }
        
        auto parts = splitString(str, ',');
        if (parts.size() != 3)
        {
            std::string err_str = 
                "invalid input: expected this format: 'x, y, z', received '" + std::string(str) + "'";
            throw RuntimeError(err_str);
        }
        else{
            geometry_msgs::Point output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            
            return output;
        }
    }

    template <> inline geometry_msgs::Quaternion convertFromString(StringView str)
    {
        if (str.size() == 0) {
            throw RuntimeError("empty stringview");
        }
        
        auto parts = splitString(str, ',');
        if (parts.size() != 4)
        {
            std::string err_str = 
                "invalid input: expected this format: 'x, y, z, w', received '" + std::string(str) + "'";
            throw RuntimeError(err_str);
        }
        else{
            geometry_msgs::Quaternion output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            output.w = convertFromString<double>(parts[3]);
            
            return output;
        }
    }


    template <> inline geometry_msgs::TransformStamped convertFromString(StringView str)
    {
        if (str.size() == 0) {
            throw RuntimeError("empty stringview");
        }
        
        // Template: ref_frame, child_frame, x, y, z, x_rot, y_rot, z_rot, w_rot
        auto parts = splitString(str, ',');
        if (parts.size() != 9)
        {
            std::string err_str = 
                "invalid input: expected this format: 'ref_frame, child_frame, x, y, z, x_rot, y_rot, z_rot, w_rot', received '" + std::string(str) + "'";
            throw RuntimeError(err_str);
        }
        else{
            geometry_msgs::TransformStamped output;
            output.header.seq = 0; //nobody use this anymore
            output.header.stamp.sec = 0;
            output.header.stamp.nsec = 0;
            output.header.frame_id = parts[0];
            output.child_frame_id = parts[1];
            
            output.transform.translation.x = convertFromString<double>(parts[2]);
            output.transform.translation.y = convertFromString<double>(parts[3]);
            output.transform.translation.z = convertFromString<double>(parts[4]);
            output.transform.rotation.x = convertFromString<double>(parts[5]);
            output.transform.rotation.y = convertFromString<double>(parts[6]);
            output.transform.rotation.z = convertFromString<double>(parts[7]);
            output.transform.rotation.w = convertFromString<double>(parts[8]);
            
            return output;
        }
    }
    
} // end namespace BT

#endif // HHCM_BT_CPP_LIBS_PORT_GEOMETRY_MSGS_CONVERSIONS
