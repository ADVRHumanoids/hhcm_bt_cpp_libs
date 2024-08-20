#ifndef HHCM_BT_CPP_LIBS_SETSTRING_ROS_SERVER_HANDLER_H
#define HHCM_BT_CPP_LIBS_SETSTRING_ROS_SERVER_HANDLER_H

#include <ros/ros.h>
#include <behaviortree_cpp/blackboard.h>

#include <xbot_msgs/SetString.h>

namespace hhcm_bt {

/**
 * This is not a BT node. It is a ROS server which will receive external requests. Once received the string,
 * the data is put in the BT GLOBAL BLACKBOARD, created externally by this object owner. The owner should also
 * remember to call the ros spin/spinonce in the loop otherwise this server callback is not called. 
 */
class SetStringRosServerHandler {
    
public:
    
    SetStringRosServerHandler(    
        ros::NodeHandle* nh, 
        const std::string& server_name, 
        BT::Blackboard::Ptr blackboard);


private:
    BT::Blackboard::Ptr blackboard;
    const std::string server_name;
    ros::ServiceServer server;
    bool clbk(xbot_msgs::SetString::Request &req, xbot_msgs::SetString::Response &res);

    
};

} //namespace


#endif //HHCM_BT_CPP_LIBS_SETSTRING_ROS_SERVER_HANDLER_H