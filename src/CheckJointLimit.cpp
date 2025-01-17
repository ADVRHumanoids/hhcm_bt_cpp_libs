#include <hhcm_bt_cpp_libs/CheckJointLimit.h>

using hhcm_bt::CheckJointLimit;

CheckJointLimit::CheckJointLimit(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{

}

BT::PortsList CheckJointLimit::providedPorts()
{
    return { 
        BT::InputPort<xbot_msgs::JointState>("joint_states"), 
        BT::InputPort<std::string>("joint_name"), 
        BT::InputPort<double>("upper_limit"),
        BT::InputPort<double>("lower_limit"),
        BT::OutputPort<JointLimitStatus>("joint_limit_status")
    };
}

BT::NodeStatus CheckJointLimit::tick()
{
    auto joint_name_exp = getInput<std::string>("joint_name");
    if (!joint_name_exp) {
        throw std::runtime_error ("ERROR CheckJointLimit: no joint_name port");
    }
    std::string joint_name = joint_name_exp.value();

    auto upper_limit_exp = getInput<double>("upper_limit"); 
    if (!upper_limit_exp) {
        throw std::runtime_error ("ERROR CheckJointLimit: no upper_limit port");
    }
    double upper_limit = upper_limit_exp.value();

    auto lower_limit_exp = getInput<double>("lower_limit"); 
    if (!lower_limit_exp) {
        throw std::runtime_error ("ERROR CheckJointLimit: no lower_limit port");
    }
    double lower_limit = lower_limit_exp.value();


    if(auto joint_states_exp = getLockedPortContent("joint_states"))
    {
        if(joint_states_exp->empty())
        {
            throw std::runtime_error("CheckJointLimit: no joint_state key found in the bb");
        }

        if(const xbot_msgs::JointState* joint_states = joint_states_exp->castPtr<xbot_msgs::JointState>())
        {
            JointLimitStatus joint_limit_status;

            for (size_t i = 1; i<joint_states->name.size(); i++) {
                if (joint_states->name[i].compare(joint_name) == 0 ) {
                    if (joint_states->link_position[i] > upper_limit) {
                        joint_limit_status = JointLimitStatus::AboveUpper;
                    } else if (joint_states->link_position[i] < lower_limit) {
                        joint_limit_status = JointLimitStatus::BelowLower;
                    } else {
                        joint_limit_status = JointLimitStatus::Inside;
                    }  
                }
            }
            setOutput("joint_limit_status", joint_limit_status);

            printf("joint_limit_status: %d\n", joint_limit_status);


        } else {
            std::cout << "[" << name() << "] CheckJointLimit running castPtr failed" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    } else {
        std::cout << "[" << name() << "] CheckJointLimit running getLockedPortContent failed" << std::endl;
        return BT::NodeStatus::FAILURE;
    }


    return BT::NodeStatus::SUCCESS;
}
