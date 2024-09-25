#ifndef HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_CLIENT_NODE_H
#define HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_CLIENT_NODE_H

#include <hhcm_bt_cpp_libs/behaviortree_ros/RosServiceNode.h>
#include <hhcm_bt_cpp_libs/PortGeometryMsgsConversions.h>
#include <hhcm_bt_cpp_libs/PortStdConversions.h>
#include <moveit_msgs/ApplyPlanningScene.h>

namespace hhcm_bt {

/**
 * This is a BT node, hence used inside the BT to call an external ros service being a ROS client
 * WARN: Do not use this, use ObjectPlanningSceneRosPubClientNode instead. This node is not 
 * updated with recents changess
 */

class ObjectPlanningSceneRosClientNode : public BT::RosServiceNode<moveit_msgs::ApplyPlanningScene> 
{

public:

    ObjectPlanningSceneRosClientNode(
        const std::string& name, const BT::NodeConfig & conf,
        ros::NodeHandle* nh, const std::string& service_name,
        const std::string& ee_link, const std::vector<std::string>& ee_touch_links);

    virtual ~ObjectPlanningSceneRosClientNode() {};

    virtual bool prepareRequest(RequestType& request) override;
    virtual bool modifyRequest(RequestType& request) override;
        
    static BT::PortsList providedPorts();
    
    virtual BT::NodeStatus onResponse( const ResponseType& res) override;

private:

    const std::string ee_link;
    const std::vector<std::string> ee_touch_links;

    std::string obj_id;
    std::string obj_type;
    std::string obj_ref_frame;
    std::vector<double> obj_size;
    geometry_msgs::Pose obj_pose;
    std::string add_to, remove_from;

    bool create_object(const std::string& id, 
                       const std::string& type,
                       const std::vector<double>& dim,
                       const geometry_msgs::Pose& pose,
                       moveit_msgs::CollisionObject& obj);

    bool get_ports();

};

} //namespace
    

#endif //HHCM_BT_CPP_LIBS_OBJECT_PLANNING_SCENE_ROS_CLIENT_NODE_H
