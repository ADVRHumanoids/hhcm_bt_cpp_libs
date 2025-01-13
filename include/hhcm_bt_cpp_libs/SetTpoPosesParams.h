#ifndef HHCM_BT_CPP_LIBS_SET_TPO_POSES_PARAMS_H
#define HHCM_BT_CPP_LIBS_SET_TPO_POSES_PARAMS_H

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/Quaternion.h>

namespace hhcm_bt
{

class SetTpoPosesParams : public BT::SyncActionNode
{
public:
    SetTpoPosesParams(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

} //namespace hhcm_bt

#endif // HHCM_BT_CPP_LIBS_SET_TPO_POSES_PARAMS_H