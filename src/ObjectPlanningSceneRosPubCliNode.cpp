#include <hhcm_bt_cpp_libs/ObjectPlanningSceneRosPubCliNode.h>

using hhcm_bt::ObjectPlanningSceneRosPubCliNode;

ObjectPlanningSceneRosPubCliNode::ObjectPlanningSceneRosPubCliNode(
    const std::string& name, const BT::NodeConfig & conf,
        ros::NodeHandle* nh, const std::string& pub_topic_name, const std::string& srv_name,
        const std::string& ee_link, const std::vector<std::string>& ee_touch_links) : 
        BT::StatefulActionNode(name, conf), 
        nh_(nh),
        pub_topic_name(pub_topic_name), srv_name(srv_name),
        ee_link(ee_link), ee_touch_links(ee_touch_links)
{
    pub_ = nh_->advertise<moveit_msgs::PlanningScene>(pub_topic_name, 1);
    srv_client_ = nh_->serviceClient<moveit_msgs::GetPlanningScene>(srv_name);
    srv_.request.components.components =  
        moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS + 
        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES + 
        moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
}

BT::PortsList ObjectPlanningSceneRosPubCliNode::providedPorts() {

    return {
        BT::InputPort<std::string>("add_to", "robot and/or world"),
        BT::InputPort<std::string>("remove_from", "robot and/or world"),
        BT::InputPort<std::vector<std::string>>("allowed_collisions_robot", "robot links to which this object is ok to collide"),
        BT::InputPort<std::vector<std::string>>("not_allowed_collisions_robot", "robot links to which this object is not ok to collide"),
        BT::InputPort<std::vector<std::string>>("allowed_collisions_world", "allow these object to collide with the object, and allowed_collisions_robot, but not with not_allowed_collisions_robot"),
        BT::InputPort<std::vector<std::string>>("not_allowed_collisions_world", "not allow these object to collide with anything (object, allowed_collisions_robot, and not_allowed_collisions_robot)"),
        BT::InputPort<std::string>("obj_id", "", "name, unique in the environment"),
        BT::InputPort<std::string>("obj_type", ""),
        BT::InputPort<std::string>("obj_ref_frame", ""),
        BT::InputPort<geometry_msgs::Pose>("obj_pose"),
        BT::InputPort<double>("obj_size_x", 0, "size in x dimension"),
        BT::InputPort<double>("obj_size_y", 0, "size in y dimension"),
        BT::InputPort<double>("obj_size_z", 0, "size in z dimension")
    };
}

BT::NodeStatus ObjectPlanningSceneRosPubCliNode::onStart() {

    msg_to_pub_ready_ = false;

    if (!get_ports()) {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ObjectPlanningSceneRosPubCliNode::onRunning() {

    if (!srv_client_.call(srv_)) {
        ROS_ERROR("FAIL to call service %s", srv_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!msg_to_pub_ready_) {
        prepareMsg();
    }

    pub_.publish(planning_scene_msg);

    if (!checkForUpdated()){
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::SUCCESS;
}

void ObjectPlanningSceneRosPubCliNode::onHalted() {
    msg_to_pub_ready_ = false;
}

bool ObjectPlanningSceneRosPubCliNode::prepareMsg() {

    if (add_to.size() > 0) {

        //ADD COLLISION OBJ
        moveit_msgs::AttachedCollisionObject attached_collision_object;
        attached_collision_object.object.id = obj_id;

        if (obj_to_create && !create_object(obj_type, obj_size, obj_pose, attached_collision_object.object))
        {
            ROS_ERROR("FAIL to create object");
            return false;
        } 
        
        if (add_to.find("robot") != std::string::npos){

            attached_collision_object.link_name = ee_link;
            attached_collision_object.object.operation = moveit_msgs::CollisionObject::ADD;
            attached_collision_object.touch_links = ee_touch_links;
            planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_collision_object);
            planning_scene_msg.robot_state.is_diff = true;

        } 
        if (add_to.find("world") != std::string::npos){
            planning_scene_msg.world.collision_objects.push_back(attached_collision_object.object);
        }         
                     
    }

    collision_detection::AllowedCollisionMatrix acm(srv_.response.scene.allowed_collision_matrix);
    
    if (remove_from.size() > 0) {

        if (remove_from.find("robot") != std::string::npos) {
            moveit_msgs::AttachedCollisionObject detach_object;
            detach_object.object.id = obj_id;
            detach_object.link_name = ee_link;
            detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
            planning_scene_msg.robot_state.attached_collision_objects.push_back(detach_object);
            planning_scene_msg.robot_state.is_diff = true;
        } 
        
        if (remove_from.find("world") != std::string::npos) {

            moveit_msgs::CollisionObject remove_object;       
            remove_object.id = obj_id;
            remove_object.header.frame_id = obj_ref_frame;
            remove_object.operation = remove_object.REMOVE;
            planning_scene_msg.world.collision_objects.push_back(remove_object);
        }

        acm.removeEntry(obj_id);

    }

    for (const auto& it: allowed_collisions_robot) {
        acm.setEntry(it, obj_id, collision_detection::AllowedCollision::ALWAYS);
    }

    for (const auto& it: not_allowed_collisions_robot) {
        acm.setEntry(it, obj_id, collision_detection::AllowedCollision::NEVER);
    }

    for (const auto& it: allowed_collisions_world) {
        acm.setEntry(it, obj_id, collision_detection::AllowedCollision::ALWAYS);
        //allow also collision between such world objects and the robots parts provided
        for (const auto& rob: allowed_collisions_robot) {
            acm.setEntry(it, rob, collision_detection::AllowedCollision::ALWAYS);
        }
        for (const auto& rob: not_allowed_collisions_robot) {
            acm.setEntry(it, rob, collision_detection::AllowedCollision::NEVER);
        }
    }

    for (const auto& it: not_allowed_collisions_world) {
        acm.setEntry(it, obj_id, collision_detection::AllowedCollision::NEVER);
        //disaallow also collision between such world objects and the robots parts provided
        for (const auto& rob: not_allowed_collisions_robot) {
            acm.setEntry(it, rob, collision_detection::AllowedCollision::NEVER);
        }
        for (const auto& rob: allowed_collisions_robot) {
            acm.setEntry(it, rob, collision_detection::AllowedCollision::NEVER);
        }
    }

    
    acm.getMessage(planning_scene_msg.allowed_collision_matrix);  

    planning_scene_msg.is_diff = true;

    msg_to_pub_ready_ = true;

    return true;
}


bool ObjectPlanningSceneRosPubCliNode::checkForUpdated() { 

    if (add_to.size() > 0) {
        
        if (add_to.find("robot") != std::string::npos) {

            bool add_to_robot_update = false;
            for (const auto& it : srv_.response.scene.robot_state.attached_collision_objects) {
                if (it.object.id.compare(obj_id) == 0) {
                    add_to_robot_update = true;
                    break;
                }
            }
            if (!add_to_robot_update) { //no sense to check for the other

                return false;
            }
        } 

        if (add_to.find("world") != std::string::npos){

            bool add_to_world_update = false;
            for (const auto& it : srv_.response.scene.world.collision_objects) { 
                if (it.id.compare(obj_id) == 0) {
                    add_to_world_update = true;
                    break;
                }
            }
            if (!add_to_world_update) { //no sense to check for the other
                return false;
            }
        }                      
    }

    if (remove_from.size() > 0) {

        if (remove_from.find("robot") != std::string::npos) {

            for (const auto& it : srv_.response.scene.robot_state.attached_collision_objects) {
                if (it.object.id.compare(obj_id) == 0) {
                    std::cout << " obj to remove robot still there" << std::endl;

                    return false; //obj to remove still there
                }
            }
        } 
        
        if (remove_from.find("world") != std::string::npos) {

            for (const auto& it : srv_.response.scene.world.collision_objects) {
                if (it.id.compare(obj_id) == 0) {
                                        std::cout << " obj to remove world still there" << std::endl;

                    return false; //obj to remove still there
                }
            }
        }
    }

    if (allowed_collisions_robot.size() > 0) {
        collision_detection::AllowedCollisionMatrix acm(srv_.response.scene.allowed_collision_matrix);
        collision_detection::AllowedCollision::Type allowed_collision_type;
        for (const auto& it: allowed_collisions_robot) {
            if (! acm.hasEntry(it, obj_id)) {
                std::cout << " acm not entry" << std::endl;

                return false;
            }
            acm.getEntry(it, obj_id, allowed_collision_type);
            if (allowed_collision_type != collision_detection::AllowedCollision::ALWAYS) {

                std::cout << " acm allowed collision not yet allowed" << std::endl;

                return false;
            }
        }
    }  

    if (not_allowed_collisions_robot.size() > 0) {
        collision_detection::AllowedCollisionMatrix acm(srv_.response.scene.allowed_collision_matrix);
        collision_detection::AllowedCollision::Type allowed_collision_type;
        for (const auto& it: not_allowed_collisions_robot) {
            if ( acm.hasEntry(it, obj_id)) { //if there is not, we consider fine since by default everything collide
                acm.getEntry(it, obj_id, allowed_collision_type);
                //but if there is, we have to make sure it is set as NEVER allowed
                if (allowed_collision_type != collision_detection::AllowedCollision::NEVER) {

                    std::cout << " acm not allowed collision not yet not allowed" << std::endl;

                    return false;
                }
            }
        }
    } 

    if (allowed_collisions_world.size() > 0) {
        collision_detection::AllowedCollisionMatrix acm(srv_.response.scene.allowed_collision_matrix);
        collision_detection::AllowedCollision::Type allowed_collision_type;
        for (const auto& it: allowed_collisions_world) {
            if (! acm.hasEntry(it, obj_id)) {
                std::cout << " acm not entry" << std::endl;

                return false;
            }
            acm.getEntry(it, obj_id, allowed_collision_type);
            if (allowed_collision_type != collision_detection::AllowedCollision::ALWAYS) {

                std::cout << " acm allowed collision not yet allowed" << std::endl;

                return false;
            }
        }
    }

    if (not_allowed_collisions_world.size() > 0) {
        collision_detection::AllowedCollisionMatrix acm(srv_.response.scene.allowed_collision_matrix);
        collision_detection::AllowedCollision::Type allowed_collision_type;
        for (const auto& it: not_allowed_collisions_world) {
            if ( acm.hasEntry(it, obj_id)) { //if there is not, we consider fine since by default everything collide
                acm.getEntry(it, obj_id, allowed_collision_type);
                //but if there is, we have to make sure it is set as NEVER allowed
                if (allowed_collision_type != collision_detection::AllowedCollision::NEVER) {

                    std::cout << " acm not allowed collision not yet not allowed" << std::endl;

                    return false;
                }
            }
        }
    }

    return true;
}

bool ObjectPlanningSceneRosPubCliNode::create_object(
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
    obj.operation = moveit_msgs::CollisionObject::ADD;

    obj.primitives.push_back(primitive);    
    obj.primitive_poses.push_back(pose);

    return true;

}

bool ObjectPlanningSceneRosPubCliNode::get_ports() {

    auto add_to_exp = getInput<std::string>("add_to");
    auto remove_from_exp = getInput<std::string>("remove_from");
    auto allowed_collisions_robot_exp = getInput<std::vector<std::string>>("allowed_collisions_robot");
    auto not_allowed_collisions_robot_exp = getInput<std::vector<std::string>>("not_allowed_collisions_robot");
    auto allowed_collisions_world_exp = getInput<std::vector<std::string>>("allowed_collisions_world");
    auto not_allowed_collisions_world_exp = getInput<std::vector<std::string>>("not_allowed_collisions_world");

    auto obj_id_exp = getInput<std::string>("obj_id");
    auto obj_type_exp = getInput<std::string>("obj_type");
    auto obj_ref_frame_exp = getInput<std::string>("obj_ref_frame");
    auto obj_pose_exp = getInput<geometry_msgs::Pose>("obj_pose");
    auto obj_size_x_exp = getInput<double>("obj_size_x");
    auto obj_size_y_exp = getInput<double>("obj_size_y");
    auto obj_size_z_exp = getInput<double>("obj_size_z");

    obj_to_create = false;

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
        ROS_ERROR("obj_id port not defined or it is empty!");
        return false;
    }
    obj_id = obj_id_exp.value();

    if (!allowed_collisions_robot_exp || allowed_collisions_robot_exp.value().size() == 0) {
        allowed_collisions_robot.clear();
    } else {
        allowed_collisions_robot = allowed_collisions_robot_exp.value();
    }    
    
    if (!not_allowed_collisions_robot_exp || not_allowed_collisions_robot_exp.value().size() == 0) {
        not_allowed_collisions_robot.clear();
    } else {
        not_allowed_collisions_robot = not_allowed_collisions_robot_exp.value();
    }

    if(!allowed_collisions_world_exp || allowed_collisions_world_exp.value().size() == 0) {
        allowed_collisions_world.clear();
    } else {
        allowed_collisions_world = allowed_collisions_world_exp.value();
    }
    if (!not_allowed_collisions_world_exp || not_allowed_collisions_world_exp.value().size() == 0) {
        not_allowed_collisions_world.clear();
    } else {
        not_allowed_collisions_world = not_allowed_collisions_world_exp.value();
    }

    if (add_to.size() > 0) {

        if (!obj_type_exp || obj_type_exp.value().size() == 0) {
            obj_type = "";
            obj_ref_frame = "";
            obj_pose = geometry_msgs::Pose();
            obj_size.clear();
            return true;
        }   
        if (!obj_ref_frame_exp || obj_ref_frame_exp.value().size() == 0) {
            ROS_ERROR("obj_ref_frame port not defined or it is empty!");
            return false;
        }       
        if (!obj_pose_exp) {
            ROS_ERROR("obj_pose port not defined");
            return false;
        }       
        if (!obj_size_x_exp) {
            ROS_ERROR("obj_size_x port not defined!");
            return false;
        }
        if (!obj_size_y_exp) {
            ROS_ERROR("obj_size_y port not defined!");
            return false;
        }
        if (!obj_size_z_exp) {
            ROS_ERROR("obj_size_z port not defined!");
            return false;
        }

        obj_to_create = true;

        obj_type = obj_type_exp.value();
        obj_ref_frame = obj_ref_frame_exp.value();
        obj_pose = obj_pose_exp.value();
        obj_size = {obj_size_x_exp.value(), obj_size_y_exp.value(), obj_size_z_exp.value()};

    } else {
        obj_type = "";
        obj_ref_frame = "";
        obj_pose = geometry_msgs::Pose();
        obj_size.clear();
    }

    return true;
}