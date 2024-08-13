#include <hhcm_bt_cpp_libs/TFHandler.h>

using hhcm_bt::TFHandler;
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

TFHandler::TFHandler(ros::NodeHandle* nh) : 
    nh(nh)
{
    
    tf_internal = std::make_shared<TF>();
    tf = tf_internal;
    
    last_time = ros::TIME_MIN;
    
    tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer, nh);
    
}


bool TFHandler::getTf()
{
    
    if ((ros::Time::now() - last_time) > ros::Duration(0.2)) { //to avoid ros warning tf repeated data
    
        try {
            
            for (auto & it: tf_internal->x_T_x) {
                
                tf2::fromMsg(
                    tf_buffer.lookupTransform(it.first.first, it.first.second, ros::Time(0)),
                    it.second);
            }
            
        }
        catch (tf2::TransformException &ex) {
            
            ROS_ERROR("TFHandler::getTF() Error: %s",ex.what());
            return false;
        }
        
        last_time = ros::Time::now();
    }
    
    return true;
}

bool TFHandler::waitForRos(const std::string &from, const std::string &to)
{

    uint count = 0;
    
    ros::Duration sleep_time(0.1);
    while (ros::ok() && count < 10) {
        
        ros::spinOnce();
    
        try {
            auto from_T_to = tf_buffer.lookupTransform(from, to, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Transform %s_T_%s NOT ready ...", from.c_str(), to.c_str());
            count++;
            sleep_time.sleep();
            continue;
        }
        ROS_INFO("Transform %s_T_%s ready!", from.c_str(), to.c_str());

        return true;
    }
    ROS_ERROR("Too much time has passed waiting for Transform %s_T_%s!", from.c_str(), to.c_str());

    return false;
}