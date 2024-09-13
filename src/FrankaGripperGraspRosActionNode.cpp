#include <hhcm_bt_cpp_libs/FrankaGripperGraspRosActionNode.h>

using hhcm_bt::FrankaGripperGraspRosActionNode;

FrankaGripperGraspRosActionNode::FrankaGripperGraspRosActionNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& server_name) :
    BT::RosActionNode<franka_gripper::GraspAction>(name, config, nh, server_name)
{
}

BT::PortsList FrankaGripperGraspRosActionNode::providedPorts() {
    
    return providedBasicPorts({ 
        BT::InputPort<double>("width"),
        BT::InputPort<double>("inner"),
        BT::InputPort<double>("outer"),
        BT::InputPort<double>("force")
    });
    
}

bool FrankaGripperGraspRosActionNode::prepareGoal(GoalType& goal) {

    auto width_exp = getInput<double>("width");
    auto inner_exp = getInput<double>("inner");
    auto outer_exp = getInput<double>("outer");
    auto force_exp = getInput<double>("force");

    if (!width_exp || !inner_exp || !outer_exp || !force_exp) {
        ROS_ERROR_STREAM("Please provide ports: 'width', 'inner', 'outer', 'force'");
        return false;
    }

    goal.width = width_exp.value();
    goal.epsilon.inner = inner_exp.value();
    goal.epsilon.outer = outer_exp.value();
    goal.force = force_exp.value();

    return true;
}

BT::NodeStatus FrankaGripperGraspRosActionNode::onResult( const ResultType& res) {

    if (res.success) {

        return BT::NodeStatus::SUCCESS;

    } else {

        ROS_ERROR("Grasp failed: %s", res.error.c_str());
        return BT::NodeStatus::FAILURE;
    }
}