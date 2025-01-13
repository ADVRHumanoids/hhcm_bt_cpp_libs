#include <hhcm_bt_cpp_libs/SetTpoPosesParams.h>

using hhcm_bt::SetTpoPosesParams;

SetTpoPosesParams::SetTpoPosesParams(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config) 
{

} 

BT::PortsList SetTpoPosesParams::providedPorts() {

    return {
        BT::BidirectionalPort<double>("offset_x", 0, ""),
        BT::BidirectionalPort<double>("offset_y", 0, ""),
        BT::BidirectionalPort<double>("offset_z", 0, ""),
        BT::BidirectionalPort<geometry_msgs::Quaternion>("offset_orientation"),
        BT::BidirectionalPort<double>("overwrite_x"),
        BT::BidirectionalPort<double>("overwrite_y"),
        BT::BidirectionalPort<double>("overwrite_z"),
        BT::BidirectionalPort<geometry_msgs::Quaternion>("overwrite_orientation"),
        BT::BidirectionalPort<uint8_t>("control_mode_x", 0, "0:SET, 1:FIX, 2:INCREMENTAL"),
        BT::BidirectionalPort<uint8_t>("control_mode_y", 0, "0:SET, 1:FIX, 2:INCREMENTAL"),
        BT::BidirectionalPort<uint8_t>("control_mode_z", 0, "0:SET, 1:FIX, 2:INCREMENTAL"),
        BT::BidirectionalPort<uint8_t>("control_mode_orientation", 0, "0:SET, 1:FIX, 2:INCREMENTAL"),
    };
}


BT::NodeStatus SetTpoPosesParams::tick() {

    auto offset_x_exp = getInput<double>("offset_x"); 
    auto offset_y_exp = getInput<double>("offset_y");
    auto offset_z_exp = getInput<double>("offset_z");
    auto offset_orientation_exp = getInput<geometry_msgs::Quaternion>("offset_orientation"); 

    auto overwrite_x_exp = getInput<double>("overwrite_x");
    auto overwrite_y_exp = getInput<double>("overwrite_y");
    auto overwrite_z_exp = getInput<double>("overwrite_z");
    auto overwrite_orientation_exp = getInput<geometry_msgs::Quaternion>("overwrite_orientation");

    auto control_mode_x_exp = getInput<uint8_t>("control_mode_x");
    auto control_mode_y_exp = getInput<uint8_t>("control_mode_y");
    auto control_mode_z_exp = getInput<uint8_t>("control_mode_z");
    auto control_mode_orientation_exp = getInput<uint8_t>("control_mode_orientation");

    if (!offset_x_exp) {
        throw BT::RuntimeError("Missing required input port: offset_x");
    }
    if (!offset_y_exp) {
        throw BT::RuntimeError("Missing required input port: offset_y");
    }
    if (!offset_z_exp) {
        throw BT::RuntimeError("Missing required input port: offset_z");
    }
    if (!offset_orientation_exp) {
        throw BT::RuntimeError("Missing required input port: offset_orientation");
    }
    if (!overwrite_x_exp) {
        throw BT::RuntimeError("Missing required input port: overwrite_x");
    }
    if (!overwrite_y_exp) {
        throw BT::RuntimeError("Missing required input port: overwrite_y");
    }
    if (!overwrite_z_exp) {
        throw BT::RuntimeError("Missing required input port: overwrite_z");
    }
    if (!overwrite_orientation_exp) {
        throw BT::RuntimeError("Missing required input port: overwrite_orientation");
    }
    if (!control_mode_x_exp) {
        throw BT::RuntimeError("Missing required input port: control_mode_x");
    }
    if (!control_mode_y_exp) {
        throw BT::RuntimeError("Missing required input port: control_mode_y");
    }
    if (!control_mode_z_exp) {
        throw BT::RuntimeError("Missing required input port: control_mode_z");
    }
    if (!control_mode_orientation_exp) {
        throw BT::RuntimeError("Missing required input port: control_mode_orientation");
    }

    config().blackboard->set<double>("offset_x", offset_x_exp.value());
    config().blackboard->set<double>("offset_y", offset_y_exp.value());
    config().blackboard->set<double>("offset_z", offset_z_exp.value());
    config().blackboard->set<geometry_msgs::Quaternion>("offset_orientation", offset_orientation_exp.value());
    config().blackboard->set<double>("overwrite_x", overwrite_x_exp.value());
    config().blackboard->set<double>("overwrite_y", overwrite_y_exp.value());
    config().blackboard->set<double>("overwrite_z", overwrite_z_exp.value());
    config().blackboard->set<geometry_msgs::Quaternion>("overwrite_orientation", overwrite_orientation_exp.value());
    config().blackboard->set<uint8_t>("control_mode_x", control_mode_x_exp.value());
    config().blackboard->set<uint8_t>("control_mode_y", control_mode_y_exp.value());
    config().blackboard->set<uint8_t>("control_mode_z", control_mode_z_exp.value());
    config().blackboard->set<uint8_t>("control_mode_orientation", control_mode_orientation_exp.value());

    return BT::NodeStatus::SUCCESS;
}
