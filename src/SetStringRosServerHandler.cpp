#include <hhcm_bt_cpp_libs/SetStringRosServerHandler.h>

using hhcm_bt::SetStringRosServerHandler;

SetStringRosServerHandler::SetStringRosServerHandler(
    ros::NodeHandle* nh, 
    const std::string& server_name, 
    BT::Blackboard::Ptr blackboard) 
    : server_name(server_name), blackboard(blackboard)
{ 

    server =    
        nh->advertiseService(
            server_name, &SetStringRosServerHandler::clbk, this);

    blackboard->set(server_name, "");
    
}

bool SetStringRosServerHandler::clbk(
    xbot_msgs::SetString::Request &req,
    xbot_msgs::SetString::Response &res){
    
    blackboard->set(server_name, req.request);
    res.success = true;
    
    return true;

}
