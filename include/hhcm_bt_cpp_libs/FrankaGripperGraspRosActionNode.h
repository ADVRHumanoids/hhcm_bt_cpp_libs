#ifndef HHCM_BT_CPP_LIBS_FRANKA_GRIPPER_GRASP_ROS_ACTION_NODE_H
#define HHCM_BT_CPP_LIBS_FRANKA_GRIPPER_GRASP_ROS_ACTION_NODE_H

#include <ros/ros.h>

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosActionNode.h>

#include <franka_gripper/GraspAction.h>

namespace hhcm_bt {

class FrankaGripperGraspRosActionNode : public BT::RosActionNode<franka_gripper::GraspAction> 
{
public:
     FrankaGripperGraspRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name);

    virtual ~FrankaGripperGraspRosActionNode() {};

    static BT::PortsList providedPorts();

    bool prepareGoal(GoalType& goal) override;
    BT::NodeStatus onResult( const ResultType& res) override;
    
};

}//namespace

#endif //  HHCM_BT_CPP_LIBS_FRANKA_GRIPPER_GRASP_ROS_ACTION_NODE_H
