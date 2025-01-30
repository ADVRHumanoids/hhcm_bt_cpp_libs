#ifndef HHCM_BT_CPP_LIBS_RESETORIENTATIONXSENSWRAPPER_ROS_CLIENT_NODE_H
#define HHCM_BT_CPP_LIBS_RESETORIENTATIONXSENSWRAPPER_ROS_CLIENT_NODE_H

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosServiceNode.h>
#include <hiros_xsens_mtw_wrapper/ResetOrientation.h>

namespace hhcm_bt {

/**
 * This is a BT node, hence used inside the BT to call an external ros service being a ROS client
 */
class ResetOrientationXSensWrapperRosClientNode : public BT::RosServiceNode<hiros_xsens_mtw_wrapper::ResetOrientation> 
{

public:

    ResetOrientationXSensWrapperRosClientNode(
        const std::string& name, const BT::NodeConfig & conf,
        ros::NodeHandle* nh, const std::string& service_name);

    virtual ~ResetOrientationXSensWrapperRosClientNode() {};

    virtual bool prepareRequest(RequestType& request) override;
        
    static BT::PortsList providedPorts();
    
    virtual BT::NodeStatus onResponse( const ResponseType& res) override;

private:

};

} //namespace
    

#endif //HHCM_BT_CPP_LIBS_RESETORIENTATIONXSENSWRAPPER_ROS_CLIENT_NODE_H
