#ifndef HHCM_BT_CPP_LIBS_TF_READER_NODE_H
#define HHCM_BT_CPP_LIBS_TF_READER_NODE_H

#include <behaviortree_cpp/action_node.h>
#include <hhcm_bt_cpp_libs/TF.h>

namespace hhcm_bt {

/**
 * This class is a BT node that simply read the given (as ports) entries
 * and look up the trasform in the hhcm_bt::TF map
 */
class TFReaderNode : public BT::SyncActionNode {

public:
    TFReaderNode(const std::string& name, const BT::NodeConfig& config, TF::Ptr tf);
    
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
private:
    TF::Ptr _tf;
    std::vector<std::string> _ref_frames;
    std::vector<std::string> _target_frames;
};

} //namespace


#endif //HHCM_BT_CPP_LIBS_TF_READER_NODE_H
