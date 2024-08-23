#ifndef HHCM_BT_CPP_LIBS_TWIST_ROS_PUB_NODE_H
#define HHCM_BT_CPP_LIBS_TWIST_ROS_PUB_NODE_H

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosPubNode.h>
#include <hhcm_bt_cpp_libs/PortGeometryMsgsConversions.h>

#include <geometry_msgs/TwistStamped.h>

namespace hhcm_bt {

class TwistRosPubNode : public BT::RosPubNode<geometry_msgs::TwistStamped> 
{
public:
     TwistRosPubNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& topic_name);

    virtual ~TwistRosPubNode() {};

    static BT::PortsList providedPorts();

    bool modifyMsg() override;

private:
    bool getPorts();
    
};

} //namespace

#endif //  HHCM_BT_CPP_LIBS_TWIST_ROS_PUB_NODE_H
