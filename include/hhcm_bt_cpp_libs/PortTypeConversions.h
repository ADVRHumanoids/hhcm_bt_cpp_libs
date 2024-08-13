#ifndef TPO_BT_PORT_TYPE_CONVERSIONS
#define TPO_BT_PORT_TYPE_CONVERSIONS

#include <behaviortree_cpp/basic_types.h> //for stringview
#include <geometry_msgs/TransformStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

// Template specialization to converts a string to geometry_msgs::transformStamped.
namespace BT
{
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
    
    struct IndicesContainer {
        bool indices[6];
        IndicesContainer() : indices{true, true, true, true, true, true} {}
    };
    
    template <> inline IndicesContainer convertFromString(StringView str)
    {
        if (str.size() == 0) {
            throw RuntimeError("Empty String, expected these formats: 'x, y, z' OR 'x, y, z, roll, pitch, yaw' OR a single character (eg '1') for all active.");
        }
                        
        IndicesContainer indices;

        auto parts = splitString(str, ',');


        if (parts.size() == 1) {
            //already set all true
            
        } else if (parts.size() == 3 || parts.size() == 6) {
            
            for (size_t i=0; i< parts.size(); i++){

//                 if (parts[i].compare("0") == 0) {
//                     indices.indices[i] = false;
//                     
//                 } else if (parts[i].compare("1") == 0) {
//                     indices.indices[i] = true;
// 
//                 } else {
//                     std::cout << "convertFromString indices error: please put 1 or 0, putted: "<< parts[i] << std::endl;
// 
//                 }
//                 parts[i].remove_prefix(std::min(parts[i].find_first_not_of(" "), parts[i].size()));
//                 
//                 parts[i].remove_suffix(parts[i].size() - parts[i].find_last_not_of(" ") - 1);
                
                indices.indices[i] = convertFromString<bool>(parts[i]);
               
            }

        } else {
                       
            std::string err_str = 
                "invalid input: expected these formats: 'x, y, z' OR 'x, y, z, roll, pitch, yaw' OR a single character (eg '1') for all active.. Received '" + std::string(str) + "'";
            throw RuntimeError(err_str);
            
        }
        
//         for (auto it : indices.indices) {
//             std::cout << it << std::endl;
//         }
        
        return indices;
    }
    
} // end namespace BT

#endif // TPO_BT_PORT_TYPE_CONVERSIONS
