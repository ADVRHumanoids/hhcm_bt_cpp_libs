#include <hhcm_bt_cpp_libs/JointVelRosPubNode.h>

using hhcm_bt::JointVelRosPubNode;

JointVelRosPubNode::JointVelRosPubNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& topic_name) :
    BT::RosPubNode<sensor_msgs::JointState>(name, config, nh, topic_name)
{

}

BT::PortsList JointVelRosPubNode::providedPorts() {
    
    return providedBasicPorts({ 
        BT::InputPort<std::vector<double>>("j_vel"),
    });
    
}

bool JointVelRosPubNode::modifyMsg() {

    return getPorts();
}

bool JointVelRosPubNode::getPorts() {

    auto j_vel_exp = getInput<std::vector<double>>("j_vel"); 

    if (! j_vel_exp)
    {
        std::cout << "JointVelRosPubNode ERROR, no j_vel port" << std::endl;
        return false;
    }     
   
    //msg_.header.frame_id = ""; 
    msg_.header.stamp = ros::Time::now();
    msg_.velocity = getInput<std::vector<double>>("j_vel").value();

    return true;
}
