#include <hhcm_bt_cpp_libs/TFToTpoPoses.h>

using hhcm_bt::TFToTpoPoses;

TFToTpoPoses::TFToTpoPoses(const std::string& name, const BT::NodeConfig& config, hhcm_bt::TF::Ptr tf) :
    BT::SyncActionNode(name, config),
    _tf(tf)
{

    auto ref_frame_exp = getInput<double>("ref_frame");
    auto target_frame_exp = getInput<double>("target_frame");

    if (!ref_frame_exp) {
        throw BT::RuntimeError("Missing required input port: ref_frame");
    }
    if (!target_frame_exp) {
        throw BT::RuntimeError("Missing required input port: target_frame");
    }

    _ref_frame = ref_frame_exp.value();
    _target_frame = target_frame_exp.value();

    if (! _tf->addTf(_ref_frame, _target_frame)) {
        throw BT::RuntimeError("add tf returned false");
    }

}

BT::PortsList TFToTpoPoses::providedPorts() {
    
    return { 
        BT::InputPort<std::string>("ref_frame"),
        BT::InputPort<std::string>("target_frame"),
        BT::InputPort<double>("offset_x", 0, ""),
        BT::InputPort<double>("offset_y", 0, ""),
        BT::InputPort<double>("offset_z", 0, ""),
        BT::InputPort<geometry_msgs::Quaternion>("offset_orientation", geometry_msgs::Quaternion(), "this will be post-multiplied to the tf"),
        BT::InputPort<double>("overwrite_x"),
        BT::InputPort<double>("overwrite_y"),
        BT::InputPort<double>("overwrite_z"),
        BT::InputPort<geometry_msgs::Quaternion>("overwrite_orientation"),
        BT::OutputPort<std::vector<geometry_msgs::Pose>>("goal_poses")
    };
    
}

BT::NodeStatus TFToTpoPoses::tick() {

    auto offset_x_exp = getInput<double>("offset_x"); 
    auto offset_y_exp = getInput<double>("offset_y");
    auto offset_z_exp = getInput<double>("offset_z");
    auto offset_orientation_exp = getInput<geometry_msgs::Quaternion>("offset_orientation"); 

    auto overwrite_x_exp = getInput<double>("overwrite_x");
    auto overwrite_y_exp = getInput<double>("overwrite_y");
    auto overwrite_z_exp = getInput<double>("overwrite_z");
    auto overwrite_orientation_exp = getInput<geometry_msgs::Quaternion>("overwrite_orientation");

    if (!_tf->getTf(_ref_frame, _target_frame, _ref_T_target, 1)) {
        return BT::NodeStatus::FAILURE;
    }

    std::vector<geometry_msgs::Pose> poses_goal;
    poses_goal.resize(1);

    if (overwrite_x_exp) {
        poses_goal.at(0).position.x = overwrite_x_exp.value();
    } else {
        if (offset_x_exp) {
            poses_goal.at(0).position.x = _ref_T_target.getOrigin().getX() + offset_x_exp.value();
        } else {
            poses_goal.at(0).position.x = _ref_T_target.getOrigin().getX();
        }
    }

    if (overwrite_y_exp) {
        poses_goal.at(0).position.y = overwrite_y_exp.value();
    } else {
        if (offset_y_exp) {
            poses_goal.at(0).position.y = _ref_T_target.getOrigin().getY() + offset_y_exp.value();
        } else {
            poses_goal.at(0).position.y = _ref_T_target.getOrigin().getY();
        }
    }

    if (overwrite_z_exp) {
        poses_goal.at(0).position.z = overwrite_z_exp.value();
    } else {
        if (offset_z_exp) {
            poses_goal.at(0).position.z = _ref_T_target.getOrigin().getZ() + offset_z_exp.value();
        } else {
            poses_goal.at(0).position.z = _ref_T_target.getOrigin().getZ();
        }
    }

    if (overwrite_orientation_exp) {
        poses_goal.at(0).orientation = overwrite_orientation_exp.value();
    } else {
        if (offset_orientation_exp) {
            auto offset_orientation = offset_orientation_exp.value();
            tf2::Quaternion quat(
                offset_orientation.x,
                offset_orientation.y,
                offset_orientation.z,
                offset_orientation.w);
            tf2::Quaternion quat_target =_ref_T_target.getRotation() * quat;
            poses_goal.at(0).orientation.x = quat_target.getX();
            poses_goal.at(0).orientation.y = quat_target.getY();
            poses_goal.at(0).orientation.z = quat_target.getZ();
            poses_goal.at(0).orientation.w = quat_target.getW();
        } else {
            poses_goal.at(0).orientation.x = _ref_T_target.getRotation().getX();
            poses_goal.at(0).orientation.y = _ref_T_target.getRotation().getY();
            poses_goal.at(0).orientation.z = _ref_T_target.getRotation().getZ();
            poses_goal.at(0).orientation.w = _ref_T_target.getRotation().getW();
        }

    }

    setOutput("goal_poses", poses_goal);

    return BT::NodeStatus::SUCCESS;
}