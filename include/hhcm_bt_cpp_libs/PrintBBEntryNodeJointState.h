#ifndef HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_NODE_JOINT_STATE_H
#define HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_NODE_JOINT_STATE_H

#include <hhcm_bt_cpp_libs/PrintBBEntryNode.h>
#include <sensor_msgs/JointState.h>

namespace hhcm_bt {

class PrintBBEntryNodeJointState : public PrintBBEntryNode<sensor_msgs::JointState>
{

public:
    PrintBBEntryNodeJointState(const std::string& name, const BT::NodeConfig& config): PrintBBEntryNode(name, config)
    {
    }

    virtual BT::NodeStatus tick() override {
        auto val_exp = getLockedPortContent("entry");

        if(val_exp->empty())
        {
            std::cout << "[" << name() << "] no variable found " << std::endl;
            return BT::NodeStatus::FAILURE;

        } else if (sensor_msgs::JointState* gripper_state = val_exp->castPtr<sensor_msgs::JointState>()) {
            std::cout << "[" << name() << "] Joint state value are: \n";
            for (int i=0; i<gripper_state->name.size(); i++) {
                std::cout << 
                    "name: " << gripper_state->name[i] << "\n" <<
                    "position: " << gripper_state->position[i] << "\n" <<
                    "velocity: " << gripper_state->velocity[i] << "\n" <<
                    "effort: " << gripper_state->effort[i] << std::endl;
            }
        } else {
            std::cout << "[" << name() << "] no variable found " << std::endl;
            return BT::NodeStatus::FAILURE;

        }
        return BT::NodeStatus::SUCCESS;
    };

};

} //namespace


#endif //HHCM_BT_CPP_LIBS_PRINT_BB_ENTRY_NODE_JOINT_STATE_H