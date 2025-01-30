#include <hhcm_bt_cpp_libs/ResetOrientationXSensWrapperRosClientNode.h>

using hhcm_bt::ResetOrientationXSensWrapperRosClientNode;

ResetOrientationXSensWrapperRosClientNode::ResetOrientationXSensWrapperRosClientNode(
        const std::string& name, const BT::NodeConfig & conf, 
        ros::NodeHandle* nh, const std::string& service_name) :
        BT::RosServiceNode<hiros_xsens_mtw_wrapper::ResetOrientation>(name, conf, nh, service_name)
{

}

BT::PortsList ResetOrientationXSensWrapperRosClientNode::providedPorts() {

    return providedBasicPorts({ 
        BT::InputPort<std::vector<std::string>>("imu_names")
    });
}


bool ResetOrientationXSensWrapperRosClientNode::prepareRequest(RequestType& request) {

    auto imu_names_exp = getInput<std::vector<std::string>>("imu_names");

    if (imu_names_exp) {
        request.sensors = imu_names_exp.value();
    } else {
        request.sensors = {};
    }

    return true;

}    

BT::NodeStatus ResetOrientationXSensWrapperRosClientNode::onResponse( const ResponseType& res) {

    if (res.success){
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}