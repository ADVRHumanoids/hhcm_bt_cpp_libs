#ifndef HHCM_BT_CPP_CHECK_JOINT_LIMIT_H
#define HHCM_BT_CPP_CHECK_JOINT_LIMIT_H

#include <behaviortree_cpp/condition_node.h>
#include <xbot_msgs/JointState.h>

namespace hhcm_bt
{

class CheckJointLimit : public BT::ConditionNode
{
public:

    enum class JointLimitStatus {
        Inside = 0,
        BelowLower,
        AboveUpper,
    };  

    CheckJointLimit(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    // Add private members here
};

} // namespace hhcm_bt

#endif // HHCM_BT_CPP_CHECK_JOINT_LIMIT_H