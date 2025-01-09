#include <hhcm_bt_cpp_libs/CheckGripperGrasp.h>

using hhcm_bt::CheckGripperGrasp;

CheckGripperGrasp::CheckGripperGrasp(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {


}

BT::PortsList CheckGripperGrasp::providedPorts() {
    return {
        BT::InputPort<sensor_msgs::JointState>("gripper_state", "{=}", ""),
        BT::InputPort<double>("requested_effort"),
        BT::InputPort<double>("requested_effort_max_err", 1, ""),
    };
}

BT::NodeStatus CheckGripperGrasp::tick() {

    auto requested_effort_exp = getInput<double>("requested_effort");
    double requested_effort;
    if (requested_effort_exp) {
        requested_effort = requested_effort_exp.value();
    } else {
        throw std::runtime_error("CheckGripperGrasp: no requested_effort key found in the bb");
    }

    auto requested_effort_max_err = getInput<double>("requested_effort_max_err");
    
    if(auto gripper_state_exp = getLockedPortContent("gripper_state"))
    {
        if(gripper_state_exp->empty())
        {
            throw std::runtime_error("CheckGripperGrasp: no gripper_state key found in the bb");
        }
        else if(const sensor_msgs::JointState* gripper_state = gripper_state_exp->castPtr<sensor_msgs::JointState>())
        {

            double err = std::abs(requested_effort - std::abs(gripper_state->effort[0])); //keep abs since dagana eff is inverted
            if (gripper_state->position[0] < 0.95 && //1.0 is open, 0.0 is closed 
                err < requested_effort_max_err.value()) {
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "[" << name() << "] Check Gripper grasp fail\n" << "Joint state value are: \n";
                for (int i=0; i<gripper_state->name.size(); i++) {
                    std::cout << 
                        "name: " << gripper_state->name[i] << "\n" <<
                        "position: " << gripper_state->position[i] << "\n" <<
                        "velocity: " << gripper_state->velocity[i] << "\n" <<
                        "effort: " << std::abs(gripper_state->effort[i]) << "\n";
                }
                std::cout << "requested_effort: " << requested_effort << "\n";
                std::cout << "effort error: " << err << "\n";
                std::cout << "requested_effort_max_err: " << requested_effort_max_err.value() << "\n" 
                    << gripper_state->position[0] << std::endl;
                }
        }
    } else {
        std::cout << "getLockedPortContent failed" << std::endl;
    }
    return BT::NodeStatus::FAILURE;

}