#include <hhcm_bt_cpp_libs/GripperRosActionNode.h>

using hhcm_bt::GripperRosActionNode;

GripperRosActionNode::GripperRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name) :
    BT::RosActionNode<control_msgs::GripperCommandAction>(name, config, nh, server_name)
{
    _position = getInput<double>("position").value();
    _max_effort = getInput<double>("max_effort").value();
}

BT::PortsList GripperRosActionNode::providedPorts() {
    
    return { 
        BT::InputPort<double>("position"),
        BT::InputPort<double>("max_effort")
    };
    
}

bool GripperRosActionNode::prepareGoal(GoalType& goal) {

    goal.command.position = _position;
    goal.command.max_effort = _max_effort;

    return true;
}

BT::NodeStatus GripperRosActionNode::onResult( const ResultType& res) {

    if (res.reached_goal || res.stalled) {

        return BT::NodeStatus::SUCCESS;

    } else {

        return BT::NodeStatus::FAILURE;
    }
}