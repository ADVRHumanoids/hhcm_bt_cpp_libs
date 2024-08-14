#ifndef HHCM_BT_CPP_LIBS_PORT_INDICES_CONVERSIONS
#define HHCM_BT_CPP_LIBS_PORT_INDICES_CONVERSIONS

#include <behaviortree_cpp/basic_types.h> //for stringview


// Template specialization to converts a string to geometry_msgs::transformStamped.
namespace BT
{
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

#endif // HHCM_BT_CPP_LIBS_PORT_INDICES_CONVERSIONS
