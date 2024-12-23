#ifndef HHCM_BT_CPP_LIBS_CHECK_GRIPPER_GRASP_H
#define HHCM_BT_CPP_LIBS_CHECK_GRIPPER_GRASP_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <behaviortree_cpp/condition_node.h>

namespace hhcm_bt {

class CheckGripperGrasp : public BT::ConditionNode {

public:
    CheckGripperGrasp(const std::string& name, const BT::NodeConfiguration& config);
    virtual ~CheckGripperGrasp() override = default;

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

} //namespace

#endif //HHCM_BT_CPP_LIBS_CHECK_GRIPPER_GRASP_H