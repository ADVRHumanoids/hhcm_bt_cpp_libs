#ifndef HHCM_BT_CPP_LIBS_PORT_STD_CONVERSIONS
#define HHCM_BT_CPP_LIBS_PORT_STD_CONVERSIONS

#include <behaviortree_cpp/basic_types.h>

// Template specialization to converts a string to geometry_msgs::transformStamped.
namespace BT
{

    template <>
    std::vector<std::string> convertFromString<std::vector<std::string>>(StringView str)
    {
        auto parts = splitString(str, ';');
        std::vector<std::string> output;
        output.reserve(parts.size());
        for(const StringView& part : parts)
        {
            output.push_back(convertFromString<std::string>(part));
        }
        return output;
    }

    //string must be in seconds
    template <> inline std::vector<std::vector<double>> convertFromString(StringView str)
    {
        
        auto parts = splitString(str, ';');
        std::vector<std::vector<double>> output;
        output.reserve(parts.size());
        for(const StringView& part : parts)
        {
            auto inner_parts = splitString(part, ',');
            std::vector<double> inner_output;
            inner_output.reserve(inner_parts.size());

            for(const StringView& inner_part : inner_parts)
            {
                inner_output.emplace_back(convertFromString<double>(inner_part));
            }

            output.emplace_back(inner_output);
        }
        return output;
    }    

    template <>
    std::string toStr(const std::vector<std::vector<double>> input) 
    {
        std::string output;

        size_t i = 0;
        for (; i<input.size()-1; i++) {
            size_t j=0;
            for (; j<input.at(i).size()-1; j++) {
                output.append(std::to_string(input.at(i).at(j)));
                output.append(",");
            }
            output.append(std::to_string(input.at(i).at(j)));
            output.append(";");
        }

        size_t j=0;
        for (; j<input.at(i).size()-1; j++) {
            output.append(std::to_string(input.at(i).at(j)));
            output.append(",");
        }
        output.append(std::to_string(input.at(i).at(j)));

        return output;
    }

    
} // end namespace BT

#endif // HHCM_BT_CPP_LIBS_PORT_STD_CONVERSIONS
