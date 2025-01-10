#ifndef HARIA_HHCM_BT_CPP_LIBS_TF_TO_TPO_POSES_H
#define HARIA_HHCM_BT_CPP_LIBS_TF_TO_TPO_POSES_H

#include <behaviortree_cpp/action_node.h>
#include <hhcm_bt_cpp_libs/TF.h>

namespace hhcm_bt
{

/**
 * @class TFToTpoPoses
 * @brief A BT node that takes ROS TF transformation (with the TFHandler) and output (as a BT port) 
 * a TPO poses object, suitable for the TpoPosesRosActionNode. the pose will be unique, so the tpo poses
 * will be of one element. 
 * Eventually, the port of this node can overrwrite certain part of the trasform, or add an offset to it
 * @warning / TODO the frames port are readed in the costructor
 */
class TFToTpoPoses : public BT::SyncActionNode
{
public:
    TFToTpoPoses(const std::string& name, const BT::NodeConfig& config, hhcm_bt::TF::Ptr tf);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    hhcm_bt::TF::Ptr _tf;
    std::string _ref_frame;
    std::string _target_frame;
    tf2::Stamped<tf2::Transform> _ref_T_target;

};

} // namespace hhcm_bt

#endif // HARIA_HHCM_BT_CPP_LIBS_TF_TO_TPO_POSES_H
