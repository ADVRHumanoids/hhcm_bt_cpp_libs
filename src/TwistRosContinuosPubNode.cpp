#include <hhcm_bt_cpp_libs/TwistRosContinuosPubNode.h>

using hhcm_bt::TwistRosContinuosPubNode;

TwistRosContinuosPubNode::TwistRosContinuosPubNode(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& topic_name) :
    BT::RosContinuosPubNode<geometry_msgs::TwistStamped>(name, config, nh, topic_name)
{

}

BT::PortsList TwistRosContinuosPubNode::providedPorts() {
    
    return providedBasicPorts({ 
        BT::InputPort<std::string>("frame_id"),
        BT::InputPort<geometry_msgs::Vector3>("linear"),
        BT::InputPort<geometry_msgs::Vector3>("angular")
    });
    
}

bool TwistRosContinuosPubNode::onStartInitialization() {

    return getPorts();
}

bool TwistRosContinuosPubNode::modifyMsg() {

    return getPorts();
}

bool TwistRosContinuosPubNode::getPorts() {

    auto frame_id_exp = getInput<std::string>("frame_id"); 
    auto linear_exp = getInput<geometry_msgs::Vector3>("linear"); 
    auto angular_exp = getInput<geometry_msgs::Vector3>("angular"); 

    if (! frame_id_exp)
    {
        ROS_ERROR_STREAM("TwistRosContinuosPubNode ERROR, no frame_id port");
        return false;
    }     
    if (! linear_exp)
    {
        ROS_ERROR_STREAM("TwistRosContinuosPubNode ERROR, no linear port");
        return false;
    }     
    if (! angular_exp)
    {
        ROS_ERROR_STREAM("TwistRosContinuosPubNode ERROR, no angular port");
        return false;
    } 

    msg_.header.frame_id = getInput<std::string>("frame_id").value();
    msg_.header.stamp = ros::Time::now();
    msg_.twist.linear = getInput<geometry_msgs::Vector3>("linear").value();
    msg_.twist.angular = getInput<geometry_msgs::Vector3>("angular").value();

    return true;
}
