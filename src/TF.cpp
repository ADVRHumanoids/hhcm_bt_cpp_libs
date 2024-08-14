#include <hhcm_bt_cpp_libs/TF.h>

using hhcm_bt::TF;

bool TF::addTf(const std::string& from, const std::string& to) {
    
    std::pair<std::string, std::string> key = std::make_pair(from, to);
    return addTf(key);
    
}
bool TF::addTf(const std::pair<std::string, std::string> & key ) {
    
    if (key.first.size() == 0) {
        ROS_ERROR_STREAM("TF::addTf ERROR: first frame is empty");
        return false;
    }
    if (key.first.size() == 0) {
        ROS_ERROR_STREAM("TF::addTf ERROR: second frame is empty");
        return false;
    }

    //stack overflow fast lookup
    auto lb = x_T_x.lower_bound(key);
    
    if(lb != x_T_x.end() && !(x_T_x.key_comp()(key, lb->first)))
    {
        
    }
    else
    {
        //std::cout << "inserting " << from << "   " << to << std::endl;
        x_T_x.insert(lb, std::map<std::pair<std::string, std::string>, tf2::Stamped<tf2::Transform> >::value_type(key, tf2::Stamped<tf2::Transform>()));
    }
    
    return true;
    
}