#include <hhcm_bt_cpp_libs/TpoJointPositionRosActionNode.h>

using hhcm_bt::TpoJointPositionRosActionNode;

TpoJointPositionRosActionNode::TpoJointPositionRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name) :
    BT::RosActionNode<tpo_msgs::JointPositionAction>(name, config, nh, server_name)
{
}

BT::PortsList TpoJointPositionRosActionNode::providedPorts() {
    
    return providedBasicPorts({ 
        BT::InputPort<std::vector<float>>("joint_position"),
        BT::InputPort<float>("error_norm", 0.01, "(rad)")
    });
    
}

bool TpoJointPositionRosActionNode::prepareGoal(GoalType& goal) {

    const std::vector<float> position = getInput<std::vector<float>>("joint_position").value();
    const float error_norm = getInput<float>("error_norm").value();

    goal.position = position;
    goal.wanted_joint_position_error_norm = error_norm;

    return true;
}

BT::NodeStatus TpoJointPositionRosActionNode::onResult( const ResultType& res) {

    if (res.actual_joint_position_error_norm < res.wanted_joint_position_error_norm) {

        return BT::NodeStatus::SUCCESS;

    } else {

        return BT::NodeStatus::FAILURE;
    }
}