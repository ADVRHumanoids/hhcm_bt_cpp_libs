#include <hhcm_bt_cpp_libs/RosHandler.h>

using hhcm_bt::TFHandler;
using hhcm_bt::TF;
using hhcm_bt::ServiceHandler;

bool TF::addTf(const std::string& from, const std::string& to) {
    
    std::pair<std::string, std::string> key = std::make_pair(from, to);
    return addTf(key);
    
}
bool TF::addTf(const std::pair<std::string, std::string> & key ) {
    
    if (key.first.size() == 0) {
        ROS_ERROR_STREAM("TF::addTf ERROR: first frame is empty");
        return false;
    }
    if (key.first.size() == 0) {
        ROS_ERROR_STREAM("TF::addTf ERROR: second frame is empty");
        return false;
    }

    //stack overflow fast lookup
    auto lb = x_T_x.lower_bound(key);
    
    if(lb != x_T_x.end() && !(x_T_x.key_comp()(key, lb->first)))
    {
        
    }
    else
    {
        //std::cout << "inserting " << from << "   " << to << std::endl;
        x_T_x.insert(lb, std::map<std::pair<std::string, std::string>, tf2::Stamped<tf2::Transform> >::value_type(key, tf2::Stamped<tf2::Transform>()));
    }
    
    return true;
    
}

TFHandler::TFHandler(ros::NodeHandle* nh) : 
    nh(nh)
{
    
    tf_internal = std::make_shared<TF>();
    tf = tf_internal;
    
    last_time = ros::TIME_MIN;
    
    tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer, nh);
    
}


bool TFHandler::getTf()
{
    
    if ((ros::Time::now() - last_time) > ros::Duration(0.2)) { //to avoid ros warning tf repeated data
    
        try {
            
            for (auto & it: tf_internal->x_T_x) {
                
//                 std::cout << it.first.first << std::endl;
//                 std::cout << it.first.second << std::endl;
//                 std::cout << std::endl;
             
                if ((it.first.first.compare(tf->goal_vision_frame) == 0) ||  (it.first.second.compare(tf->goal_vision_frame) == 0 )) {
                    continue;
                }
                if ((it.first.first.compare(tf->object_frame) == 0) ||  (it.first.second.compare(tf->object_frame) == 0 )) {
                    continue;
                }
                
                tf2::fromMsg(
                    tf_buffer.lookupTransform(it.first.first, it.first.second, ros::Time(0)),
                    it.second);
            }
            
        }
        catch (tf2::TransformException &ex) {
            
            ROS_ERROR("TFHandler::getTF() Error: %s",ex.what());
            return false;
        }
        
        last_time = ros::Time::now();
    }
    
    return true;
}

bool TFHandler::waitForRos() {

    uint count = 0;
    
    ros::Duration sleep_time(0.1);
    while (ros::ok() && count < 10) {
        
        ros::spinOnce();
    
        try{
            auto ref_T_robot = tf_buffer.lookupTransform(tf->ref_frame, tf->body_frame, ros::Time(0));
            auto foot_T_body = tf_buffer.lookupTransform(tf->footprint_frame, tf->body_frame, ros::Time(0));

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Transforms not ready...");
            count++;
            sleep_time.sleep();
            continue;
        }
        ROS_WARN("Transforms ready!");

        return true;
    }
    
    return false;
}


/************************* */



ServiceHandler::ServiceHandler(ros::NodeHandle* nh) : nh(nh)
{
    
    server_data_internal = std::make_shared<ServerData>();   
    server_data = server_data_internal;
    
    servers.push_back(       
        nh->advertiseService(
            "gripper_grasp", &ServiceHandler::daganaGraspClbk, this)
    );
    servers.push_back(       
        nh->advertiseService(
            "gripper_open", &ServiceHandler::daganaOpenClbk, this)
    );
    servers.push_back(       
        nh->advertiseService(
            "gripper_close", &ServiceHandler::daganaCloseClbk, this)
    );
    servers.push_back(       
        nh->advertiseService(
            "tracking", &ServiceHandler::trackingClbk, this)
    );
    
    
}

bool ServiceHandler::daganaGraspClbk(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res){
    
    server_data_internal->gripper_grasp_srv = req.data;
    res.success = true;
    
    return true;

}
bool ServiceHandler::daganaOpenClbk(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res){
    
    server_data_internal->gripper_open_srv = req.data;
    res.success = true;
    
    return true;

}
bool ServiceHandler::daganaCloseClbk(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res){
    
    server_data_internal->gripper_close_srv = req.data;
    res.success = true;
    
    return true;

}
bool ServiceHandler::trackingClbk(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res){
    
    server_data_internal->tracking_srv = req.data;
    res.success = true;
    
    return true;

}

