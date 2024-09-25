#ifndef HHCM_BT_CPP_LIBS_OBJECTCOMPUTEGRASPINGOFFSET_H
#define HHCM_BT_CPP_LIBS_OBJECTCOMPUTEGRASPINGOFFSET_H

#include <behaviortree_cpp/action_node.h>
#include <hhcm_bt_cpp_libs/TF.h>

namespace hhcm_bt {

class ObjectComputeGraspingOffset : public BT::SyncActionNode {
public:
    ObjectComputeGraspingOffset(
        const std::string& name, 
        const BT::NodeConfig& config,
        hhcm_bt::TF::Ptr tf);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    std::string ee_frame;
    std::string ref_frame;
    hhcm_bt::TF::Ptr _tf;
};

} // namespace hhcm_bt_cpp_libs

#endif // HHCM_BT_CPP_LIBS_OBJECTCOMPUTEGRASPINGOFFSET_H