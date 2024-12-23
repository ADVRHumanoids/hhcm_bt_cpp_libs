#ifndef HHCM_BT_CPP_LIBS_ROS_SUB_HANDLER_H
#define HHCM_BT_CPP_LIBS_ROS_SUB_HANDLER_H

#include <ros/ros.h>
#include <behaviortree_cpp/blackboard.h>


namespace hhcm_bt {

/**
 * This is not a BT node. It is a ROS sub wrapper which will sub to a callback. Once received the message,
 * the data is put in the BT GLOBAL BLACKBOARD, created externally by this object owner. The owner should also
 * remember to call the ros spin/spinonce in the loop otherwise this callback is not called. 
 */
template <typename SubMsgType>
class RosSubHandler {
    
public:
    
    RosSubHandler(    
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
        sub = nh->subscribe<SubMsgType>(topic_name, 1, &RosSubHandler::clbk, this);
        ROS_INFO_STREAM("Waiting for message on topic " << topic_name);
        last_msg = *(ros::topic::waitForMessage<SubMsgType>(topic_name));
        ROS_INFO_STREAM("... message on topic " << topic_name << " received");

        blackboard->set(port_name, last_msg);
    };

protected:
    virtual void clbk(const typename SubMsgType::ConstPtr& msg) {
        last_msg = *msg;
        blackboard->set(port_name, last_msg);
    }

private:
    ros::NodeHandle* nh;
    BT::Blackboard::Ptr blackboard;
    const std::string topic_name;
    const std::string port_name;
    SubMsgType last_msg;
    ros::Subscriber sub;

};

} //namespace


#endif //HHCM_BT_CPP_LIBS_ROS_SUB_HANDLER_H


