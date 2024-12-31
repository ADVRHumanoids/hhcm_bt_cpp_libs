#ifndef HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_NODE_ARRAY_H
#define HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_NODE_ARRAY_H

#include <behaviortree_cpp/action_node.h>

namespace hhcm_bt {

class PrintBBEntryNodeArray : public BT::SyncActionNode
{

public:
    PrintBBEntryNodeArray(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::array<double,3>>("entry")};
    }

    virtual BT::NodeStatus tick() override {
        auto val_exp = getInput<std::array<double,3>>("entry");

        if(val_exp)
        {
            std::array<double,3> array = val_exp.value();
            std::cout << "[" << name() << "] Array values are: \n";
            for (int i=0; i<array.size()-1; i++) {
                std::cout << array.at(i) << "; ";
            }
            std::cout << array.back() << std::endl;

            
        } else {
            std::cout << "[" << name() << "] val_exp is empty found " << std::endl;
            return BT::NodeStatus::FAILURE;

        }
        return BT::NodeStatus::SUCCESS;
    };

};

} //namespace


#endif //HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_NODE_ARRAY_H