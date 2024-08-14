#include <hhcm_bt_cpp_libs/TwistRosPubNode.h>

using hhcm_bt::TwistRosPubNode;

TwistRosPubNode::TwistRosPubNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& topic_name) :
    BT::RosPubNodeContinuos<geometry_msgs::TwistStamped>(name, config, nh, topic_name)
{

}

BT::PortsList TwistRosPubNode::providedPorts() {
    
    return { 
        BT::InputPort<std::string>("frame_id"),
        BT::InputPort<geometry_msgs::Vector3>("linear"),
        BT::InputPort<geometry_msgs::Vector3>("angular")
    };
    
}

bool TwistRosPubNode::onStartInitialization() {

    getPorts();

    return true;
}

bool TwistRosPubNode::modifyMsg() {

    getPorts();

    return true;
}

void TwistRosPubNode::getPorts() {

    auto frame_id_exp = getInput<std::string>("frame_id"); 

    if (frame_id_exp)
    {
        pub_msg_.header.frame_id = getInput<std::string>("frame_id").value();
    } 

    pub_msg_.header.stamp = ros::Time::now();
    pub_msg_.twist.linear = getInput<geometry_msgs::Vector3>("linear").value();
    pub_msg_.twist.angular = getInput<geometry_msgs::Vector3>("angular").value();
}
