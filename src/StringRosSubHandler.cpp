#include <hhcm_bt_cpp_libs/StringRosSubHandler.h>

using hhcm_bt::StringRosSubHandler;

StringRosSubHandler::StringRosSubHandler(
    ros::NodeHandle* nh, 
    const std::string& topic_name, 
    const std::string& port_name, 
    BT::Blackboard::Ptr blackboard)
    :  nh(nh), topic_name(topic_name), port_name(port_name), blackboard(blackboard)
{ 

    //USING LAMBRA CAUSES main_bt: ../nptl/pthread_mutex_lock.c:428: __pthread_mutex_lock_full: Assertion `e != ESRCH || !robust' failed. 
    //c++ 14 capture (https://stackoverflow.com/questions/7895879/using-member-variable-in-lambda-capture-list-inside-a-member-function)
    // sub = nh->subscribe<std_msgs::String>(topic_name, 1,             
    //     [&blackboard = blackboard, port_name=port_name](const boost::shared_ptr<std_msgs::String const> msg) { 
    //         std::cout << "AAAAAAAAA" << std::endl;
    //         std::cout << "msg->data  " << msg->data << std::endl;
    //         std::cout << "port_name  " << port_name << std::endl;
    //         blackboard->set(port_name, msg->data);
    //     }
    // );         
    sub = nh->subscribe<std_msgs::String>(topic_name, 1, &StringRosSubHandler::clbk, this);
    last_msg = "";      

    blackboard->set(port_name, "");
}

void StringRosSubHandler::clbk(const std_msgs::String::ConstPtr& msg) {
    if (last_msg.compare(msg->data) != 0) {
        blackboard->set(port_name, msg->data);
        last_msg = msg->data;
    }
}

