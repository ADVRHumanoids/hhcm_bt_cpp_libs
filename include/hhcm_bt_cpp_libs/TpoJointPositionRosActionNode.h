#ifndef HHCM_BT_CPP_LIBS_TPO_JOINT_POSITION_ROS_ACTION_NODE_H
#define HHCM_BT_CPP_LIBS_TPO_JOINT_POSITION_ROS_ACTION_NODE_H

#include <ros/ros.h>

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosActionNode.h>
#include <hhcm_bt_cpp_libs/PortGeometryMsgsConversions.h>
#include <tpo_msgs/JointPositionAction.h>

namespace hhcm_bt {

class TpoJointPositionRosActionNode : public BT::RosActionNode<tpo_msgs::JointPositionAction> 
{
public:
     TpoJointPositionRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name);

    virtual ~TpoJointPositionRosActionNode() {};

    static BT::PortsList providedPorts();

    bool prepareGoal(GoalType& goal) override;
    BT::NodeStatus onResult( const ResultType& res) override;

private:
    
};

}//namespace

#endif //  HHCM_BT_CPP_LIBS_TPO_JOINT_POSITION_ROS_ACTION_NODE_H
