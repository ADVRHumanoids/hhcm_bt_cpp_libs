#include <hhcm_bt_cpp_libs/TpoPosesRosActionNode.h>

using hhcm_bt::TpoPosesRosActionNode;

TpoPosesRosActionNode::TpoPosesRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name) :
    BT::RosActionNode<tpo_msgs::PosesAction>(name, config, nh, server_name)
{
}

BT::PortsList TpoPosesRosActionNode::providedPorts() {
    
    return providedBasicPorts({ 
        BT::InputPort<std::string>("goal_frame_id"),
        BT::InputPort<std::vector<geometry_msgs::Pose>>("goal_poses"),
        BT::InputPort<float>("error_pos_norm", 0.01, "(m)"),
        BT::InputPort<float>("error_rot_norm", 0.05, "(rad)"),
        BT::InputPort<uint8_t>("control_mode_x", 0, "0:SET, 1:FIX, 2:INCREMENTAL"),
        BT::InputPort<uint8_t>("control_mode_y", 0, "0:SET, 1:FIX, 2:INCREMENTAL"),
        BT::InputPort<uint8_t>("control_mode_z", 0, "0:SET, 1:FIX, 2:INCREMENTAL"),
        BT::InputPort<uint8_t>("control_mode_orientation", 0, "0:SET, 1:FIX, 2:INCREMENTAL"),     
    });
    
}

bool TpoPosesRosActionNode::prepareGoal(GoalType& goal) {

    const std::string frame_id = getInput<std::string>("goal_frame_id").value();
    const std::vector<geometry_msgs::Pose> poses = getInput<std::vector<geometry_msgs::Pose>>("goal_poses").value();

    const float error_pos_norm = getInput<float>("error_pos_norm").value();
    const float error_rot_norm = getInput<float>("error_rot_norm").value();
    
    uint8_t control_mode_x, control_mode_y, control_mode_z, control_mode_orientation;
    if (!getInput<uint8_t>("control_mode_x", control_mode_x)) {
        throw BT::RuntimeError("Missing required input port: control_mode_x");
    }
    if (!getInput<uint8_t>("control_mode_y", control_mode_y)) {
        throw BT::RuntimeError("Missing required input port: control_mode_y");
    }
    if (!getInput<uint8_t>("control_mode_z", control_mode_z)) {
        throw BT::RuntimeError("Missing required input port: control_mode_z");
    }
    if (!getInput<uint8_t>("control_mode_orientation", control_mode_orientation)) {
        throw BT::RuntimeError("Missing required input port: control_mode_orientation");
    }

    
    goal.header.frame_id = frame_id;
    goal.target_poses = poses;
    goal.wanted_position_error_norm = error_pos_norm;
    goal.wanted_rotation_error_norm = error_rot_norm;
    goal.control_mode_x = control_mode_x;
    goal.control_mode_y = control_mode_y;
    goal.control_mode_z = control_mode_z;
    goal.control_mode_orientation = control_mode_orientation;

    return true;
}

BT::NodeStatus TpoPosesRosActionNode::onResult( const ResultType& res) {

    if (res.actual_position_error_norm < res.wanted_position_error_norm || 
        res.actual_rotation_error_norm < res.wanted_rotation_error_norm) {

        return BT::NodeStatus::SUCCESS;

    } else {

        std::cout << "TpoPosesRosActionNode failed: actual pos error: " << res.actual_position_error_norm << ", wanted: " << res.wanted_position_error_norm << 
            ", actual rot error: " << res.actual_rotation_error_norm << ", wanted: " << res.wanted_rotation_error_norm << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}