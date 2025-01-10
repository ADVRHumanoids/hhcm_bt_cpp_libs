#ifndef HHCM_BT_CPP_LIBS_CHECK_GRIPPER_GRASP_H
#define HHCM_BT_CPP_LIBS_CHECK_GRIPPER_GRASP_H

#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <behaviortree_cpp/action_node.h>

namespace hhcm_bt {

class CheckGripperGrasp : public BT::StatefulActionNode
{

public:
    CheckGripperGrasp(const std::string& name, const BT::NodeConfiguration& config);
    virtual ~CheckGripperGrasp() override = default;

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double requested_effort;
    double requested_effort_max_err;
    int msec;
    double joint_vel_lim;
    std::chrono::system_clock::time_point deadline;
};

} //namespace

#endif //HHCM_BT_CPP_LIBS_CHECK_GRIPPER_GRASP_H