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

        for(StringView& part : parts)
        {
            part.remove_prefix(std::min(part.find_first_not_of(" "), part.size()));
            part.remove_prefix(std::min(part.find_first_not_of("\n"), part.size()));
            part.remove_prefix(std::min(part.find_first_not_of("\t"), part.size()));
            part.remove_suffix(part.size() - std::min(part.find_last_not_of(" "), part.size()) - 1);
            part.remove_suffix(part.size() - std::min(part.find_last_not_of("\n"), part.size()) - 1);
            part.remove_suffix(part.size() - std::min(part.find_last_not_of("\t"), part.size()) - 1);
            output.push_back(convertFromString<std::string>(part));
        }
        return output;
    }

    template <>
    std::array<double, 3> convertFromString<std::array<double, 3>>(StringView str)
    {
        auto parts = splitString(str, ';');
        std::array<double, 3> output;

        unsigned int i = 0;
        for(StringView& part : parts)
        {
            part.remove_prefix(std::min(part.find_first_not_of(" "), part.size()));
            part.remove_prefix(std::min(part.find_first_not_of("\n"), part.size()));
            part.remove_prefix(std::min(part.find_first_not_of("\t"), part.size()));
            part.remove_suffix(part.size() - std::min(part.find_last_not_of(" "), part.size()) - 1);
            part.remove_suffix(part.size() - std::min(part.find_last_not_of("\n"), part.size()) - 1);
            part.remove_suffix(part.size() - std::min(part.find_last_not_of("\t"), part.size()) - 1);
            output[i] = convertFromString<double>(part);
            i++;
        }
        return output;
    }

    template <>
    std::vector<float> convertFromString<std::vector<float>>(StringView str)
    {
        auto parts = splitString(str, ';');
        std::vector<float> output;
        output.resize(parts.size());

        unsigned int i = 0;
        for(StringView& part : parts)
        {
            part.remove_prefix(std::min(part.find_first_not_of(" "), part.size()));
            part.remove_prefix(std::min(part.find_first_not_of("\n"), part.size()));
            part.remove_prefix(std::min(part.find_first_not_of("\t"), part.size()));
            part.remove_suffix(part.size() - std::min(part.find_last_not_of(" "), part.size()) - 1);
            part.remove_suffix(part.size() - std::min(part.find_last_not_of("\n"), part.size()) - 1);
            part.remove_suffix(part.size() - std::min(part.find_last_not_of("\t"), part.size()) - 1);
            output[i] = convertFromString<float>(part);
            i++;
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
    std::string toStr(const std::vector<std::vector<double>>& input) 
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

    template <>
    std::string toStr(const std::vector<std::string>& input) 
    {
        std::string output;

        size_t i = 0;
        for (; i<input.size()-1; i++) {
            output.append(input.at(i));
            output.append(";");
        }

        //append last without ;
        output.append(input.at(i));

        return output;
    }

    template <>
    std::string toStr(const std::array<double, 3>& input) 
    {
        std::string output;

        size_t i = 0;
        for (; i<input.size()-1; i++) {
            output.append(std::to_string(input.at(i)));
            output.append(";");
        }

        //append last without ;
        output.append(std::to_string(input.at(i)));

        return output;
    }

    template <>
    std::string toStr(const std::vector<float>& input) 
    {
        std::string output;

        size_t i = 0;
        for (; i<input.size()-1; i++) {
            output.append(std::to_string(input.at(i)));
            output.append(";");
        }

        //append last without ;
        output.append(std::to_string(input.at(i)));

        return output;
    }

    
} // end namespace BT

#endif // HHCM_BT_CPP_LIBS_PORT_STD_CONVERSIONS
