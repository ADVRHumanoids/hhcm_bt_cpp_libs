#ifndef HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_H
#define HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_H

#include <behaviortree_cpp/action_node.h>

namespace hhcm_bt {

template<typename Type>
class PrintBBEntryNode : public BT::SyncActionNode
{
public:
    PrintBBEntryNode<Type>(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<Type>("entry")};
    }

    BT::NodeStatus tick() override
    {
        auto val_exp = getInput<Type>("entry");

        if (val_exp) {
            std::cout << "[" << name() << "] entry value is: " << val_exp.value() << std::endl;
        } else {
            std::cout << "[" << name() << "] no variable found " << std::endl;

        }
        return BT::NodeStatus::SUCCESS;
    }
};

} //namespace

#endif //HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_H