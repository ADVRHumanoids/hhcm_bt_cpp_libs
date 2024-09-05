#ifndef HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_PUB_CLI_NODE_H
#define HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_PUB_CLI_NODE_H

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>
#include <mutex>

#include <hhcm_bt_cpp_libs/PortGeometryMsgsConversions.h>
#include <hhcm_bt_cpp_libs/PortStdConversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>

namespace hhcm_bt {

/**
 * This is a BT node
 */
class ObjectPlanningSceneRosPubCliNode : 
    public BT::StatefulActionNode
{

public:

    ObjectPlanningSceneRosPubCliNode(
        const std::string& name, const BT::NodeConfig & conf,
        ros::NodeHandle* nh, const std::string& pub_topic_name, const std::string& srv_name,
        const std::string& ee_link, const std::vector<std::string>& ee_touch_links);

    virtual ~ObjectPlanningSceneRosPubCliNode() {};

    virtual BT::NodeStatus onStart() override;
    virtual BT::NodeStatus onRunning() override;
    virtual void onHalted() override;

    static BT::PortsList providedPorts();

private:

    ros::NodeHandle* nh_;
    std::string pub_topic_name, srv_name;
    ros::Publisher pub_;
    moveit_msgs::PlanningScene planning_scene_msg;
    bool msg_to_pub_ready_;
    ros::ServiceClient srv_client_;
    moveit_msgs::GetPlanningScene srv_;

    const std::string ee_link;
    const std::vector<std::string> ee_touch_links;

    std::string obj_id;
    std::string obj_type;
    std::string obj_ref_frame;
    std::vector<double> obj_size;
    geometry_msgs::Pose obj_pose;
    std::string add_to, remove_from;
    std::vector<std::string> allowed_collisions;
    std::vector<std::string> not_allowed_collisions;

    bool prepareMsg();
    bool checkForUpdated();

    bool create_object(const std::string& id, 
                       const std::string& type,
                       const std::vector<double>& dim,
                       const geometry_msgs::Pose& pose,
                       moveit_msgs::CollisionObject& obj);

    bool get_ports();

};

} //namespace
    

#endif //HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_PUB_CLI_NODE_H
