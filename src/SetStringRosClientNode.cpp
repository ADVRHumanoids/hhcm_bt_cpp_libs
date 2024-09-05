#include <hhcm_bt_cpp_libs/SetStringRosClientNode.h>

using hhcm_bt::SetStringRosClientNode;

SetStringRosClientNode::SetStringRosClientNode(
        const std::string& name, const BT::NodeConfig & conf, 
        ros::NodeHandle* nh, const std::string& service_name) :
        BT::RosServiceNode<xbot_msgs::SetString>(name, conf, nh, service_name)
{

}

BT::PortsList SetStringRosClientNode::providedPorts() {

    return providedBasicPorts({ 
        BT::InputPort<std::string>("request"),
        BT::OutputPort<std::string>("requested")
    });
}


bool SetStringRosClientNode::prepareRequest(RequestType& request) {

    _the_string = getInput<std::string>("request").value();

    request.request = _the_string;

    return true;

}    

BT::NodeStatus SetStringRosClientNode::onResponse( const ResponseType& res) {

    if (res.success){
        setOutput("requested", _the_string);
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}