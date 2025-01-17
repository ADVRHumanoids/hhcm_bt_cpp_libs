#include <hhcm_bt_cpp_libs/CheckJointsLimits.h>

using hhcm_bt::CheckJointsLimits;

CheckJointsLimits::CheckJointsLimits(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{

}

BT::PortsList CheckJointsLimits::providedPorts()
{
    return { 
        BT::InputPort<xbot_msgs::JointState>("joint_state"), 
        BT::InputPort<std::vector<double>>("upper_limits"),
        BT::InputPort<std::vector<double>>("lower_limits"),
        BT::OutputPort<std::vector<JointLimitStatus>>("joint_limit_status")
    };
}

BT::NodeStatus CheckJointsLimits::tick()
{
    auto upper_limits_exp = getInput<std::vector<double>>("upper_limits"); 
    if (!upper_limits_exp) {
        throw std::runtime_error ("ERROR CheckJointsLimits: no upper_limits port");
    }
    std::vector<double> upper_limits = upper_limits_exp.value();

    auto lower_limits_exp = getInput<std::vector<double>>("lower_limits"); 
    if (!lower_limits_exp) {
        throw std::runtime_error ("ERROR CheckJointsLimits: no lower_limits port");
    }
    std::vector<double> lower_limits = lower_limits_exp.value();


    if(auto joint_state_exp = getLockedPortContent("joint_state"))
    {
        if(joint_state_exp->empty())
        {
            throw std::runtime_error("CheckJointsLimits: no joint_state key found in the bb");
        }

        if(const xbot_msgs::JointState* joint_state = joint_state_exp->castPtr<xbot_msgs::JointState>())
        {
            std::vector<JointLimitStatus> joint_limit_status;
            joint_limit_status.resize(joint_state->name.size());

            for (size_t i = 1; i<joint_state->name.size(); i++) {
                if (joint_state->link_position[i] > upper_limits[i]) {
                    joint_limit_status[i] = JointLimitStatus::AboveUpper;
                } else if (joint_state->link_position[i] < lower_limits[i]) {
                    joint_limit_status[i] = JointLimitStatus::BelowLower;
                } else {
                    joint_limit_status[i] = JointLimitStatus::Inside;
                }  
            }
            setOutput("joint_limit_status", joint_limit_status);

            for (int i = 1; i<joint_limit_status.size(); i++) {
                printf("joint_limit_status[%d] : %d\n",i,joint_limit_status[i]);
            }


        } else {
            std::cout << "[" << name() << "] CheckJointsLimits running castPtr failed" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    } else {
        std::cout << "[" << name() << "] CheckJointsLimits running getLockedPortContent failed" << std::endl;
        return BT::NodeStatus::FAILURE;
    }


    return BT::NodeStatus::SUCCESS;
}
