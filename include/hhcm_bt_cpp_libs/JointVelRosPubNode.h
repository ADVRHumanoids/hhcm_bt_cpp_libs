#ifndef HHCM_BT_CPP_LIBS_JOINT_VEL_ROS_PUB_NODE_H
#define HHCM_BT_CPP_LIBS_JOINT_VEL_ROS_PUB_NODE_H

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosPubNode.h>
#include <hhcm_bt_cpp_libs/PortGeometryMsgsConversions.h>

#include <sensor_msgs/JointState.h>

namespace hhcm_bt {

class JointVelRosPubNode : public BT::RosPubNode<sensor_msgs::JointState> 
{
public:
     JointVelRosPubNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& topic_name);

    virtual ~JointVelRosPubNode() {};

    static BT::PortsList providedPorts();

    bool modifyMsg() override;

private:
    bool getPorts();
    
};

} //namespace

#endif //  HHCM_BT_CPP_LIBS_JOINT_VEL_ROS_PUB_NODE_H
