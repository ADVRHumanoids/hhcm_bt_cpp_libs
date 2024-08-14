#include <hhcm_bt_cpp_libs/SetStringRosServiceNode.h>

using hhcm_bt::SetStringRosServiceNode;

SetStringRosServiceNode::SetStringRosServiceNode(
        const std::string& name, const BT::NodeConfig & conf, 
        ros::NodeHandle* nh, const std::string& service_name) :
        BT::RosServiceNode<xbot_msgs::SetString>(name, conf, nh, service_name)
{

}

BT::PortsList SetStringRosServiceNode::providedPorts() {

    return { 
        BT::InputPort<double>("request"),
        BT::InputPort<unsigned>("timeout", 100, "timeout to connect to server (milliseconds)")
    };
}


bool SetStringRosServiceNode::prepareRequest(RequestType& request) {

    _the_string = getInput<std::string>("request").value();

    request.request = _the_string;

    return true;

}    

BT::NodeStatus SetStringRosServiceNode::onResponse( const ResponseType& res) {

    if (res.success){
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}