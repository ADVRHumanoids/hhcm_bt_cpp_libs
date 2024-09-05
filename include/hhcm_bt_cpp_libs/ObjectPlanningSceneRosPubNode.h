#ifndef HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_PUB_NODE_H
#define HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_PUB_NODE_H

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosPubAndWaitNode.h>
#include <behaviortree_cpp/bt_factory.h>
#include <hhcm_bt_cpp_libs/PortGeometryMsgsConversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>

namespace hhcm_bt {

/**
 * 
 * This is a BT node. 
 * @warning Not really to be used beause the acm overwrites the actual one, check ObjectPlanningSceneRosPubSubNode instead
 */
class ObjectPlanningSceneRosPubNode : 
    public BT::RosPubAndWaitNode<moveit_msgs::PlanningScene> 
{

public:

    ObjectPlanningSceneRosPubNode(
        const std::string& name, const BT::NodeConfig & conf,
        ros::NodeHandle* nh, const std::string& pub_topic_name,
        const std::string& ee_link, const std::vector<std::string>& ee_touch_links);

    virtual ~ObjectPlanningSceneRosPubNode() {};

    virtual bool prepareMsg() override;

    static BT::PortsList providedPorts();

private:

    const std::string ee_link;
    const std::vector<std::string> ee_touch_links;

    std::string obj_id;
    std::string obj_type;
    std::string obj_ref_frame;
    std::vector<double> obj_size;
    geometry_msgs::Pose obj_pose;
    std::string add_to, remove_from;
    std::vector<std::string> allowed_collisions;


    bool create_object(const std::string& id, 
                       const std::string& type,
                       const std::vector<double>& dim,
                       const geometry_msgs::Pose& pose,
                       moveit_msgs::CollisionObject& obj);

    bool get_ports();

};

} //namespace
    

#endif //HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_PUB_NODE_H
