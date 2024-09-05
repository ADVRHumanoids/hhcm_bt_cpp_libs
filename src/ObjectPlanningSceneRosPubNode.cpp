#include <hhcm_bt_cpp_libs/ObjectPlanningSceneRosPubNode.h>

using hhcm_bt::ObjectPlanningSceneRosPubNode;

ObjectPlanningSceneRosPubNode::ObjectPlanningSceneRosPubNode(
    const std::string& name, const BT::NodeConfig & conf,
        ros::NodeHandle* nh, const std::string& pub_topic_name,
        const std::string& ee_link, const std::vector<std::string>& ee_touch_links) : 
        BT::RosPubAndWaitNode<moveit_msgs::PlanningScene> (name, conf, nh, pub_topic_name),
        ee_link(ee_link), ee_touch_links(ee_touch_links)
{


}

BT::PortsList ObjectPlanningSceneRosPubNode::providedPorts() {

    return providedBasicPorts({
        BT::InputPort<std::string>("add_to", "robot and/or world"),
        BT::InputPort<std::string>("remove_from", "robot and/or world"),
        BT::InputPort<std::vector<std::string>>("allowed_collisions", "robot links to which this object is ok to collide"),
        BT::InputPort<std::string>("obj_id", "name, unique in the environment"),
        BT::InputPort<std::string>("obj_type"),
        BT::InputPort<std::string>("obj_ref_frame"),
        BT::InputPort<geometry_msgs::Pose>("obj_pose"),
        BT::InputPort<std::vector<double>>("obj_size")
    });
}


bool ObjectPlanningSceneRosPubNode::prepareMsg() {

    get_ports();

    moveit_msgs::PlanningScene planning_scene_msg;

    if (add_to.size() > 0) {

        //ADD COLLISION OBJ
        moveit_msgs::AttachedCollisionObject attached_collision_object;

        if (! create_object(obj_id, obj_type, obj_size, obj_pose, attached_collision_object.object))
        {
            return false;
        } 
        
        if (add_to.find("robot") != std::string::npos){

            attached_collision_object.link_name = ee_link;
            attached_collision_object.object.operation = moveit_msgs::CollisionObject::ADD;
            attached_collision_object.touch_links = ee_touch_links;
            planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_collision_object);
            planning_scene_msg.robot_state.is_diff = true;

        } 
        else if (add_to.find("world") != std::string::npos){
            planning_scene_msg.world.collision_objects.push_back(attached_collision_object.object);
        }         
                     
    }

    if (remove_from.find("robot") != std::string::npos) {
        moveit_msgs::AttachedCollisionObject detach_object;
        detach_object.object.id = obj_id;
        detach_object.link_name = ee_link;
        detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
        planning_scene_msg.robot_state.attached_collision_objects.push_back(detach_object);
        planning_scene_msg.robot_state.is_diff = true;
        
    } else if (remove_from.find("world") != std::string::npos) {

        moveit_msgs::CollisionObject remove_object;       
        remove_object.id = obj_id;
        remove_object.header.frame_id = obj_ref_frame;
        remove_object.operation = remove_object.REMOVE;
        planning_scene_msg.world.collision_objects.push_back(remove_object);
    }

    if (allowed_collisions.size() > 0) {
        collision_detection::AllowedCollisionMatrix acm;
        for (const auto& it: allowed_collisions) {
            acm.setEntry(it, obj_id, collision_detection::AllowedCollision::ALWAYS);
        }
        acm.getMessage(planning_scene_msg.allowed_collision_matrix);  
    }  



    planning_scene_msg.is_diff = true;
    pub_msg_ = planning_scene_msg;

    return true;
}

bool ObjectPlanningSceneRosPubNode::create_object(const std::string& id, 
                                           const std::string& type,
                                           const std::vector<double>& dim,
                                           const geometry_msgs::Pose& pose,
                                           moveit_msgs::CollisionObject& obj) 
{
    shape_msgs::SolidPrimitive primitive;

    if (type.compare("box") == 0) {
        if (dim.size() != 3) {
            ROS_ERROR("AddObject FAIL: type (%s) and dimension's size (%ld) mismatch", type.c_str(), dim.size());
            return false;
        }  
        primitive.type = shape_msgs::SolidPrimitive::BOX;

    } else if (type.compare("sphere") == 0) { 
        if (dim.size() != 1) {
            ROS_ERROR("AddObject FAIL: type (%s) and dimension's size (%ld) mismatch", type.c_str(), dim.size());
            return false;
        }  
        primitive.type = shape_msgs::SolidPrimitive::SPHERE;

    } else if (type.compare("cylinder") == 0) { 
        if (dim.size() != 2) {
            ROS_ERROR("AddObject FAIL: type (%s) and dimension's size (%ld) mismatch", type.c_str(), dim.size());
            return false;
        }  
        primitive.type = shape_msgs::SolidPrimitive::CYLINDER;

    } else if (type.compare("cone") == 0) { 
        if (dim.size() != 2) {
            ROS_ERROR("AddObject FAIL: type (%s) and dimension's size (%ld) mismatch", type.c_str(), dim.size());
            return false;
        }  
        primitive.type = shape_msgs::SolidPrimitive::CONE;

    } else {
        ROS_ERROR("AddObject FAIL: type (%s) not available", type.c_str());
        return false;
    }

    primitive.dimensions = dim;

    obj.header.frame_id = obj_ref_frame;
    obj.header.stamp = ros::Time::now();
    obj.id = id;
    obj.operation = moveit_msgs::CollisionObject::ADD;

    obj.primitives.push_back(primitive);    
    obj.primitive_poses.push_back(pose);

    return true;

}

bool ObjectPlanningSceneRosPubNode::get_ports() {

    auto add_to_exp = getInput<std::string>("add_to");
    auto remove_from_exp = getInput<std::string>("remove_from");
    auto allowed_collisions_exp = getInput<std::vector<std::string>>("allowed_collisions");

    auto obj_id_exp = getInput<std::string>("obj_id");
    auto obj_type_exp = getInput<std::string>("obj_type");
    auto obj_ref_frame_exp = getInput<std::string>("obj_ref_frame");
    auto obj_pose_exp = getInput<geometry_msgs::Pose>("obj_pose");
    auto obj_size_exp = getInput<std::vector<double>>("obj_size");

    if (add_to_exp && add_to_exp.value().size() > 0) {
        add_to = add_to_exp.value();
        if (add_to.find("world") == std::string::npos && add_to.find("robot") == std::string::npos) {
            throw BT::RuntimeError("Invalid Port add_to, must contains 'world' and/or 'robot'");
        }
    } else {

        add_to = "";
    }
    if (remove_from_exp && remove_from_exp.value().size() > 0) {
        remove_from = remove_from_exp.value();
        if (remove_from.find("world") == std::string::npos && remove_from.find("robot") == std::string::npos) {
            throw BT::RuntimeError("Invalid Port remove_from, must contains 'world' and/or 'robot'");
        }
    } else {
        remove_from = "";
    }

    

    if (!obj_id_exp || obj_id_exp.value().size() == 0) {
        throw BT::RuntimeError("obj_id port not defined or it is empty!");
    }
    obj_id = obj_id_exp.value();

    if (!allowed_collisions_exp || allowed_collisions_exp.value().size() == 0) {
        allowed_collisions.clear();
    } else {
        allowed_collisions = allowed_collisions_exp.value();
    }

    if (add_to.size() > 0) {

        if (!obj_type_exp || obj_type_exp.value().size() == 0) {
            throw BT::RuntimeError("obj_type port not defined or it is empty!");
        }   
        if (!obj_ref_frame_exp || obj_ref_frame_exp.value().size() == 0) {
            throw BT::RuntimeError("obj_ref_frame port not defined or it is empty!");
        }       
        if (!obj_pose_exp) {
            throw BT::RuntimeError("obj_pose port not defined");
        }       
        if (!obj_size_exp) {
            throw BT::RuntimeError("obj_pose port not defined!");
        }

        obj_type = obj_type_exp.value();
        obj_ref_frame = obj_ref_frame_exp.value();
        obj_pose = obj_pose_exp.value();
        obj_size = obj_size_exp.value();

    } else {
        obj_type = "";
        obj_ref_frame = "";
        obj_pose = geometry_msgs::Pose();
        obj_size.clear();
    }

    return true;
}