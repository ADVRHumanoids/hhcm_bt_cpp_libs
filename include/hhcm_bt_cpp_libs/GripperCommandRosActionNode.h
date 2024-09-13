#ifndef HHCM_BT_CPP_LIBS_GRIPPER_ROS_ACTION_NODE_H
#define HHCM_BT_CPP_LIBS_GRIPPER_ROS_ACTION_NODE_H

#include <ros/ros.h>

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosActionNode.h>

#include <control_msgs/GripperCommandAction.h>

namespace hhcm_bt {

class GripperCommandRosActionNode : public BT::RosActionNode<control_msgs::GripperCommandAction> 
{
public:
     GripperCommandRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name);

    virtual ~GripperCommandRosActionNode() {};

    static BT::PortsList providedPorts();

    bool prepareGoal(GoalType& goal) override;
    BT::NodeStatus onResult( const ResultType& res) override;
    
};

}//namespace

#endif //  HHCM_BT_CPP_LIBS_GRIPPER_ROS_ACTION_NODE_H
