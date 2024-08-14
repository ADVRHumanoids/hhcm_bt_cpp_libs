#ifndef HHCM_BT_CPP_LIBS_SETSTRING_ROS_SERVICE_NODE_H
#define HHCM_BT_CPP_LIBS_SETSTRING_ROS_SERVICE_NODE_H

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosServiceNode.h>
#include <xbot_msgs/SetString.h>

namespace hhcm_bt {

class SetStringRosServiceNode : public BT::RosServiceNode<xbot_msgs::SetString> 
{

public:

    SetStringRosServiceNode(
        const std::string& name, const BT::NodeConfig & conf,
        ros::NodeHandle* nh, const std::string& service_name);

    virtual ~SetStringRosServiceNode() {};

    virtual bool prepareRequest(RequestType& request) override;
        
    static BT::PortsList providedPorts();
    
    virtual BT::NodeStatus onResponse( const ResponseType& res) override;

private:
    std::string _the_string;

};

} //namespace
    

#endif //HHCM_BT_CPP_LIBS_SETSTRING_ROS_SERVICE_NODE_H
