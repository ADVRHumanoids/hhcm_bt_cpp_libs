#include <hhcm_bt_cpp_libs/CheckGripperGrasp.h>

using hhcm_bt::CheckGripperGrasp;

CheckGripperGrasp::CheckGripperGrasp(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {


}

BT::PortsList CheckGripperGrasp::providedPorts() {
    return {
        BT::InputPort<sensor_msgs::JointState>("gripper_state", "{=}", ""),
        BT::InputPort<double>("requested_effort"),
        BT::InputPort<double>("requested_effort_max_err"),
        BT::InputPort<double>("joint_vel_lim", 0.1, "joint velocity under which we start checking the condition, since otherwisw the gripper is cosndiered still moving"),
        BT::InputPort<int>("msec", 0, "how much time (in ms) to check for the condition")
    };
}

BT::NodeStatus CheckGripperGrasp::onStart() {

    auto requested_effort_exp = getInput<double>("requested_effort");
    if (requested_effort_exp) {
        requested_effort = requested_effort_exp.value();
    } else {
        throw std::runtime_error("CheckGripperGrasp: no requested_effort key found in the bb");
    }

    auto requested_effort_max_err_exp = getInput<double>("requested_effort_max_err");
    if (requested_effort_max_err_exp) {
        requested_effort_max_err = requested_effort_max_err_exp.value();
    } else {
        throw std::runtime_error("CheckGripperGrasp: no requested_effort_max_err key found in the bb");
    }
    auto msec_exp = getInput<int>("msec");
    msec = msec_exp ? msec_exp.value() : 0;

    auto joint_vel_lim_exp = getInput<double>("joint_vel_lim");
    joint_vel_lim = joint_vel_lim_exp ? joint_vel_lim_exp.value() : 0.1;
    
    deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(msec);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckGripperGrasp::onRunning() {
    
    if(auto gripper_state_exp = getLockedPortContent("gripper_state"))
    {
        if(gripper_state_exp->empty())
        {
            throw std::runtime_error("CheckGripperGrasp: no gripper_state key found in the bb");
        }

        if(const sensor_msgs::JointState* gripper_state = gripper_state_exp->castPtr<sensor_msgs::JointState>())
        {

            if (gripper_state->velocity[0] >= joint_vel_lim) {
                std::cout << "[" << name() << "] Check Gripper grasp wait for the gripper joint to stop, velocity is" <<
                    gripper_state->velocity[0] << std::endl; 
                deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(msec);
                return BT::NodeStatus::RUNNING;
            }

            //we consider error only if the torque is below the given
            bool above_torque_req = std::abs(gripper_state->effort[0]) > (std::abs(requested_effort) - requested_effort_max_err);
            if (gripper_state->position[0] < 0.95 && //0 is open, 1.0 is closed 
                above_torque_req) 
            {
                if (std::chrono::system_clock::now() >= deadline ) {
                    return BT::NodeStatus::SUCCESS;

                } else {
                    std::cout << "[" << name() << "] Check Gripper grasp running\n" << "Joint state value are: \n";
                    for (int i=0; i<gripper_state->name.size(); i++) {
                        std::cout << 
                            "name: " << gripper_state->name[i] << "\n" <<
                            "position: " << gripper_state->position[i] << "\n" <<
                            "velocity: " << gripper_state->velocity[i] << "\n" <<
                            "effort: " << std::abs(gripper_state->effort[i]) << "\n";
                    }
                    std::cout << "requested_effort: " << requested_effort << "\n";
                    std::cout << "requested_effort_max_err (above is fine): " << requested_effort_max_err << "\n" 
                        << gripper_state->position[0] << std::endl;

                    return BT::NodeStatus::RUNNING;
                }

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
                std::cout << "requested_effort_max_err (above is fine): " << requested_effort_max_err << "\n" 
                    << gripper_state->position[0] << std::endl;
                }
        } else {
            std::cout << "[" << name() << "] Check Gripper grasp running castPtr failed" << std::endl;
        }
    } else {
         std::cout << "[" << name() << "] Check Gripper grasp running getLockedPortContent failed" << std::endl;
    }
    return BT::NodeStatus::FAILURE;

}

void CheckGripperGrasp::onHalted() {
    std::cout << "[" << name() << "] Check Gripper grasp halted" << std::endl;
}