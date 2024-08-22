#include <hhcm_bt_cpp_libs/GripperRosActionNode.h>

using hhcm_bt::GripperRosActionNode;

GripperRosActionNode::GripperRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name) :
    BT::RosActionNode<control_msgs::GripperCommandAction>(name, config, nh, server_name)
{
}

BT::PortsList GripperRosActionNode::providedPorts() {
    
    return { 
        BT::InputPort<double>("position"),
        BT::InputPort<double>("max_effort"),
        BT::InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")
    };
    
}

bool GripperRosActionNode::prepareGoal(GoalType& goal) {

    auto position_exp = getInput<double>("position");
    auto max_effort_exp = getInput<double>("max_effort");

    if (!position_exp && !max_effort_exp) {
        ROS_ERROR_STREAM("Please provide at least one between 'position' and 'max_effort' args");
        return false;
    }

    if (position_exp) {
        goal.command.position = position_exp.value();
    }
    if (max_effort_exp) {
        goal.command.max_effort = max_effort_exp.value();
    }

    return true;
}

BT::NodeStatus GripperRosActionNode::onResult( const ResultType& res) {

    if (res.reached_goal || res.stalled) {

        return BT::NodeStatus::SUCCESS;

    } else {

        return BT::NodeStatus::FAILURE;
    }
}