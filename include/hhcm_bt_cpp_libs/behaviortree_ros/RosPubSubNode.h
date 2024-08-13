// taken from https://github.com/BehaviorTree/BehaviorTree.ROS,
// but using the class StatefulActionNode and not directly the ActionNodeBase

#ifndef HHCM_BT_CPP_LIBS_ROS_PUBSUB_NODE_HPP_
#define HHCM_BT_CPP_LIBS_ROS_PUBSUB_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

template<typename PubMsgType, typename SubMsgType>
class RosPubSubNode : public BT::StatefulActionNode
{
protected:
    
    RosPubSubNode(const std::string& name, const BT::NodeConfig & conf, ros::NodeHandle* nh,
                  const std::string& pub_topic_name, const std::string& sub_topic_name, const bool& pub_once=false):
        BT::StatefulActionNode(name, conf), node_(nh)
    {
        pub_ = node_->advertise<PubMsgType>(pub_topic_name, 10);
        
        //c++ 14 capture (https://stackoverflow.com/questions/7895879/using-member-variable-in-lambda-capture-list-inside-a-member-function)
        sub_ = node_->subscribe<SubMsgType>(sub_topic_name, 10,             
            [&sub_msg_ = sub_msg_](const boost::shared_ptr<SubMsgType const> msg) { 
                sub_msg_ = *msg;
            }
        ); 
    }
    
    SubMsgType getSubMsg() {return sub_msg_;};
    PubMsgType pub_msg_;    

public:

    RosPubSubNode() = delete;

    virtual ~RosPubSubNode() = default;
    
    // aliases just for the RegisterRosPubSub at the bottom
    using PubMsgTypeReg = PubMsgType;
    using SubMsgTypeReg = SubMsgType;

    /// These ports will be added automatically if this Node is
    /// registered using RegisterRosAction<DeriveClass>()
    static PortsList providedPorts() 
    {
        return  {
            BT::InputPort<unsigned>("timeout", 500, "timeout to wait for message in the sub topic (milliseconds)")
        };
    };

    ///Method called in the onStart(). can be used to the fields of  the pub_msg_ object, 
    ///and fill the field
    virtual bool onStartInitialization() = 0;
    
    ///This method is called at each loop, and can be used to modify the msg and publish a different one
    /// (like for sending a velocity reference based on the actual position). You may want to check
    /// for sub_msg_ as a feedback. OPTIONAL, as default it does nothing.
    virtual bool modifyMsg() {
        return true;
    }

    /// This method is called every time in the onRunning. 
    /// If return true, onResult is called, 
    /// if false, the node will still run in the next tick
    virtual bool hasFinished() = 0;
    
    /// Method called if hasfinished return true. can be overriden.
    /// Nodes ends if this return Success of Failure
    virtual NodeStatus onResult() 
    {
        return BT::NodeStatus::SUCCESS;
    }

    enum FailureCause{
        SUB_TOPIC_NO_MSG = 0,
        FAIL_ON_START = 1,
        FAIL_MODIFY_MSG = 2
    };

    /// Called if a fail happens.
    virtual NodeStatus onFailed(FailureCause failure)
    {
        printf("Fail to start with failure : %d\n", failure);
        std::cout << "Fail" << std::endl;

        return NodeStatus::FAILURE;
    }
  
    virtual BT::NodeStatus onStart() override final
    {
        unsigned msec = getInput<unsigned>("timeout").value();
        ros::Duration timeout(static_cast<double>(msec) * 1e-3);

        auto msg_ptr = ros::topic::waitForMessage<SubMsgType>(sub_.getTopic(), *node_, timeout);
        if( !msg_ptr ){
            std::cout << "wait for message failed on topic " << sub_.getTopic() << " after waiting for " << timeout.toSec() << std::endl;
            return onFailed(SUB_TOPIC_NO_MSG);
        }
        
        if (!onStartInitialization()) {
            return onFailed(FAIL_ON_START);
        }
                        
        return NodeStatus::RUNNING;
    }
    
    virtual BT::NodeStatus onRunning() override final
    {

        // RUNNING
        if (hasFinished()) {
            return onResult( );
        }

        if (! modifyMsg()) {
            return onFailed(FAIL_MODIFY_MSG);
        }
        pub_.publish(pub_msg_);
        
        return NodeStatus::RUNNING;
    }
    
    // Pub last message and halt. You may want to override to modify the last mesage
    // but call RosPubSubNode::onHalted after 
    virtual void onHalted() override {
        std::cout << "HALTING" << std::endl;

        pub_.publish(pub_msg_);
    }


private:

    ros::Publisher pub_;
    ros::Subscriber sub_;
    SubMsgType sub_msg_;
    
    ros::NodeHandle* node_;
      
};

/**************************************************************** /
 * derived from above, this will return always running (excpept if halted)
 * also by pub once is false (ie modify msg is called, and must be implemented)
 */
template<typename PubMsgType, typename SubMsgType>
class RosPubSubNodeContinuos: public RosPubSubNode<PubMsgType, SubMsgType> {
    
public:
    RosPubSubNodeContinuos(const std::string& name, const BT::NodeConfig & conf, ros::NodeHandle* nh,
                  const std::string& pub_topic_name, const std::string& sub_topic_name):
                  
        RosPubSubNode<PubMsgType, SubMsgType>(name, conf, nh, pub_topic_name, sub_topic_name)
        {}
        
    virtual ~RosPubSubNodeContinuos(){};
        
    virtual bool modifyMsg() = 0;
    
    virtual bool hasFinished() override final {
        return false;
    };
    
};
        

}  // namespace BT

#endif  // HHCM_BT_CPP_LIBS_ROS_PUBSUB_NODE_HPP_

