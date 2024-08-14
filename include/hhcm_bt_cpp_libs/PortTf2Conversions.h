#ifndef HHCM_BT_CPP_LIBS_PORT_TF2_CONVERSIONS
#define HHCM_BT_CPP_LIBS_PORT_TF2_CONVERSIONS

#include <behaviortree_cpp/basic_types.h> //for stringview

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

// Template specialization to converts a string to geometry_msgs::transformStamped.
namespace BT
{
    
    template <> tf2::Transform convertFromString(StringView str)
    {
        if (str.size() == 0) {
            throw RuntimeError("Empty String, expected these formats: 'x, y, z, x_rot, y_rot, z_rot, w_rot' OR 'x, y, z' OR 'x, y, z, roll, pitch, yaw' OR 'x_rot, y_rot, z_rot, w_rot' OR a single character (eg '0') for identity transformation.");
        }
                
        tf2::Transform tf;
        tf.setIdentity();   
        auto parts = splitString(str, ',');
        
        if (parts.size() == 1) {
            //already set identity
            
        } else if (parts.size() == 3) {
            
            tf.setOrigin(tf2::Vector3(convertFromString<double>(parts[0]), convertFromString<double>(parts[1]), convertFromString<double>(parts[2])));
            
        } else if (parts.size() == 6) {
            
            tf.setOrigin(tf2::Vector3(convertFromString<double>(parts[0]), convertFromString<double>(parts[1]), convertFromString<double>(parts[2])));
            
            auto quat = tf2::Quaternion();
            quat.setRPY(convertFromString<double>(parts[3]),convertFromString<double>(parts[4]),convertFromString<double>(parts[5]));
            tf.setRotation(quat);
                        
        } else if (parts.size() == 7) {
            
            tf.setOrigin(tf2::Vector3(convertFromString<double>(parts[0]), convertFromString<double>(parts[1]), convertFromString<double>(parts[2])));
            
            tf.setRotation(tf2::Quaternion(convertFromString<double>(parts[3]),
                                           convertFromString<double>(parts[4]),
                                           convertFromString<double>(parts[5]),
                                           convertFromString<double>(parts[6])));
            
        } else {
                       
            std::string err_str = 
                "invalid input: expected these formats: 'x, y, z, x_rot, y_rot, z_rot, w_rot' OR 'x, y, z' OR 'x, y, z, roll, pitch, yaw' OR 'x_rot, y_rot, z_rot, w_rot' OR a single character (eg '0') for identity transformation. Received '" + std::string(str) + "'";
            throw RuntimeError(err_str);
            
        }
        
        return tf;
    }
    
    template <> inline tf2::Stamped<tf2::Transform> convertFromString(StringView str)
    {

        if (str.size() == 0) {
            throw RuntimeError("Empty String, expected these formats: 'x, y, z, x_rot, y_rot, z_rot, w_rot' OR 'x, y, z' OR 'x, y, z, roll, pitch, yaw' OR 'x_rot, y_rot, z_rot, w_rot' OR a single character (eg '0') for identity transformation.");
        }
                
        tf2::Stamped<tf2::Transform> tf;
        tf.setIdentity();   
        auto parts = splitString(str, ',');
        
        if (parts.size() == 1) {
//             already set identity
            
        } else if (parts.size() == 3) {
            
            tf.setOrigin(tf2::Vector3(convertFromString<double>(parts[0]), convertFromString<double>(parts[1]), convertFromString<double>(parts[2])));
            
        } else if (parts.size() == 6) {
            
            tf.setOrigin(tf2::Vector3(convertFromString<double>(parts[0]), convertFromString<double>(parts[1]), convertFromString<double>(parts[2])));
            
            auto quat = tf2::Quaternion();
            quat.setRPY(convertFromString<double>(parts[3]),convertFromString<double>(parts[4]),convertFromString<double>(parts[5]));
            tf.setRotation(quat);
                        
        } else if (parts.size() == 7) {
            
            tf.setOrigin(tf2::Vector3(convertFromString<double>(parts[0]), convertFromString<double>(parts[1]), convertFromString<double>(parts[2])));
            
            tf.setRotation(tf2::Quaternion(convertFromString<double>(parts[3]),
                                           convertFromString<double>(parts[4]),
                                           convertFromString<double>(parts[5]),
                                           convertFromString<double>(parts[6])));
            
        } else {
                       
            std::string err_str = 
                "invalid input: expected these formats: 'x, y, z, x_rot, y_rot, z_rot, w_rot' OR 'x, y, z' OR 'x, y, z, roll, pitch, yaw' OR 'x_rot, y_rot, z_rot, w_rot' OR a single character (eg '0') for identity transformation. Received '" + std::string(str) + "'";
            throw RuntimeError(err_str);
            
        }
        
        return tf;
    }
    
} // end namespace BT

#endif // HHCM_BT_CPP_LIBS_PORT_TF2_CONVERSIONS
