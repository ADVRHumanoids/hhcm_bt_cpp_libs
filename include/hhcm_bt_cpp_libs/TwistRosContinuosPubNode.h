#ifndef HHCM_BT_CPP_LIBS_TWIST_ROS_CONTINUOS_PUB_NODE_H
#define HHCM_BT_CPP_LIBS_TWIST_ROS_CONTINUOS_PUB_NODE_H

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosContinuosPubNode.h>
#include <hhcm_bt_cpp_libs/PortGeometryMsgsConversions.h>

#include <geometry_msgs/TwistStamped.h>

namespace hhcm_bt {

class TwistRosContinuosPubNode : public BT::RosContinuosPubNode<geometry_msgs::TwistStamped> 
{
public:
     TwistRosContinuosPubNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& topic_name);

    virtual ~TwistRosContinuosPubNode() {};

    static BT::PortsList providedPorts();

    bool onStartInitialization() override;
    bool modifyMsg() override;

private:
    bool getPorts();
    
};

} //namespace

#endif //  HHCM_BT_CPP_LIBS_TWIST_ROS_CONTINUOS_PUB_NODE_H
