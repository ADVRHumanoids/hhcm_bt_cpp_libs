#ifndef HHCM_BT_CPP_LIBS_STRING_ROS_SUB_HANDLER_H
#define HHCM_BT_CPP_LIBS_STRING_ROS_SUB_HANDLER_H

#include <ros/ros.h>
#include <behaviortree_cpp/blackboard.h>

#include <std_msgs/String.h>

namespace hhcm_bt {

/**
 * This is not a BT node. It is a ROS sub wrapper which will sub to a callback. Once received the string,
 * the data is put in the BT GLOBAL BLACKBOARD, created externally by this object owner. The owner should also
 * remember to call the ros spin/spinonce in the loop otherwise this callback is not called. 
 */
class StringRosSubHandler {
    
public:
    
    StringRosSubHandler(    
        ros::NodeHandle* nh, 
        const std::string& topic_name, 
        const std::string& port_name, 
        BT::Blackboard::Ptr blackboard);


private:
    ros::NodeHandle* nh;
    BT::Blackboard::Ptr blackboard;
    const std::string topic_name;
    const std::string port_name;
    std::string last_msg;
    ros::Subscriber sub;
    void clbk(const std_msgs::String::ConstPtr& msg);


};

} //namespace


#endif //HHCM_BT_CPP_LIBS_STRING_ROS_SUB_HANDLER_H