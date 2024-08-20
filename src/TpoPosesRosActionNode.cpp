#include <hhcm_bt_cpp_libs/TpoPosesRosActionNode.h>

using hhcm_bt::TpoPosesRosActionNode;

TpoPosesRosActionNode::TpoPosesRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name) :
    BT::RosActionNode<tpo_msgs::PosesAction>(name, config, nh, server_name)
{
}

BT::PortsList TpoPosesRosActionNode::providedPorts() {
    
    return { 
        BT::InputPort<std::string>("goal_frame_id"),
        BT::InputPort<std::vector<geometry_msgs::Pose>>("goal_poses"),
        BT::InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")
    };
    
}

bool TpoPosesRosActionNode::prepareGoal(GoalType& goal) {

    const std::string frame_id = getInput<std::string>("goal_frame_id").value();
    const std::vector<geometry_msgs::Pose> poses = getInput<std::vector<geometry_msgs::Pose>>("goal_poses").value();

    goal.header.frame_id = frame_id;
    goal.target_poses = poses;

    return true;
}

BT::NodeStatus TpoPosesRosActionNode::onResult( const ResultType& res) {

    if (res.actual_position_error_norm <  res.wanted_position_error_norm || 
        res.actual_position_error_norm < 0.1) {

        return BT::NodeStatus::SUCCESS;

    } else {

        return BT::NodeStatus::FAILURE;
    }
}