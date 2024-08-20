#ifndef HHCM_BT_CPP_LIBS_SETSTRING_ROS_CLIENT_NODE_H
#define HHCM_BT_CPP_LIBS_SETSTRING_ROS_CLIENT_NODE_H

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosServiceNode.h>
#include <xbot_msgs/SetString.h>

namespace hhcm_bt {

/**
 * This is a BT node, hence used inside the BT to call an external ros service being a ROS client
 */
class SetStringRosClientNode : public BT::RosServiceNode<xbot_msgs::SetString> 
{

public:

    SetStringRosClientNode(
        const std::string& name, const BT::NodeConfig & conf,
        ros::NodeHandle* nh, const std::string& service_name);

    virtual ~SetStringRosClientNode() {};

    virtual bool prepareRequest(RequestType& request) override;
        
    static BT::PortsList providedPorts();
    
    virtual BT::NodeStatus onResponse( const ResponseType& res) override;

private:
    std::string _the_string;

};

} //namespace
    

#endif //HHCM_BT_CPP_LIBS_SETSTRING_ROS_CLIENT_NODE_H
