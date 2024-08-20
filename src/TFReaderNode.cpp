#include <hhcm_bt_cpp_libs/TFReaderNode.h>

using hhcm_bt::TFReaderNode;

TFReaderNode::TFReaderNode(const std::string& name, const BT::NodeConfig& config, TF::Ptr tf)
    : BT::SyncActionNode(name, config), _tf(tf)
{

    auto ref_frames_exp = getInput<std::vector<std::string>>("ref_frames"); 
    auto target_frames_exp = getInput<std::vector<std::string>>("target_frames"); 

    if (!ref_frames_exp) {
        throw BT::RuntimeError("ref_frames port is empty!");
    }        
    if (!target_frames_exp) {
        throw BT::RuntimeError("target_frames port is empty!");
    }

    _ref_frames = ref_frames_exp.value();
    _target_frames = target_frames_exp.value();

    if (_ref_frames.size() != _target_frames.size()) {

        throw BT::RuntimeError("ref_frames and target_frames must have same size!");
    }

    for (size_t i=0; i<_ref_frames.size(); i++ ) {
        if (! _tf->addTf(_ref_frames.at(i), _target_frames.at(i))) {
            throw BT::RuntimeError("add tf returned false");
        }
    }

}
    
BT::PortsList TFReaderNode::providedPorts()
{
    return { BT::InputPort<std::vector<std::string>>("ref_frames"),
                BT::InputPort<std::vector<std::string>>("target_frames"),
                BT::OutputPort<std::vector<std::string>>("transforms")
    };
}

BT::NodeStatus TFReaderNode::tick()
{
    std::vector<tf2::Stamped<tf2::Transform>> transforms;
    transforms.resize(_ref_frames.size());

    for (size_t i=0; i<_ref_frames.size(); i++ ) {

        if (! _tf->getTf(std::make_pair(_ref_frames.at(i), _target_frames.at(i)), transforms.at(i), 1.5)) {
            return BT::NodeStatus::FAILURE;
        }
    }

    setOutput("transforms", transforms);

    return BT::NodeStatus::SUCCESS;
}
