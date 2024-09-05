#ifndef HHCM_BT_CPP_LIBS_ROS_PUBANDWAIT_NODE_HPP_
#define HHCM_BT_CPP_LIBS_ROS_PUBANDWAIT_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

template<typename PubMsgType>
class RosPubAndWaitNode : public BT::StatefulActionNode
{
protected:
    
    RosPubAndWaitNode(const std::string& name, const BT::NodeConfig & conf, ros::NodeHandle* nh,
                  const std::string& pub_topic_name):
        BT::StatefulActionNode(name, conf), node_(nh)
    {
        pub_ = node_->advertise<PubMsgType>(pub_topic_name, 10);
    }
    
    PubMsgType pub_msg_;    

public:

    RosPubAndWaitNode() = delete;

    virtual ~RosPubAndWaitNode() = default;
    
    using PubMsgTypeReg = PubMsgType;

    /**
     * @brief Any subclass of RosTopicPubNode that has additinal ports must provide a
     * providedPorts method and call providedBasicPorts in it.
     *
     * @param addition Additional ports to add to BT port list
     * @return PortsList Containing basic ports along with node-specific ports
     */
    static PortsList providedBasicPorts(PortsList addition)
    {
        PortsList basic = {
            BT::InputPort<unsigned>("wait_time", 1000, "how much to wait after publishing before returning success (msec)")
        };
        basic.insert(addition.begin(), addition.end());
        return basic;
    }

    /**
     * @brief Creates list of BT ports
     * @return PortsList Containing basic ports along with node-specific ports
     */
    static PortsList providedPorts() 
    {
        return providedBasicPorts({});
    };

    ///Method called in the onStart(). can be used to the fields of  the pub_msg_ object, 
    ///and fill the field
    virtual bool prepareMsg() = 0;

  
    virtual BT::NodeStatus onStart() override final
    {
        auto wait_time_exp = getInput<unsigned>("wait_time");
        if (!wait_time_exp) {
            ROS_ERROR_STREAM("FAIL: port wait_time not found");
            return NodeStatus::FAILURE;
        }
        unsigned msec = wait_time_exp.value();
        
        if (!prepareMsg()) {
           return NodeStatus::FAILURE;
        }

        pub_.publish(pub_msg_);

        deadline_ = ros::WallTime::now() + ros::WallDuration(static_cast<double>(msec) * 1e-3);
           
        return NodeStatus::RUNNING;
    }
    
    virtual BT::NodeStatus onRunning() override final
    {
        if ( ros::WallTime::now() >= deadline_ ) {
            return NodeStatus::SUCCESS;
        }
        pub_.publish(pub_msg_);

        return NodeStatus::RUNNING;
            
    }
    
    // Pub last message and halt. 
    virtual void onHalted() override {
        std::cout << "HALTING" << std::endl;
    }


private:

    ros::Publisher pub_;
    
    ros::NodeHandle* node_;

    ros::WallTime deadline_;

      
};


}  // namespace BT

#endif  // HHCM_BT_CPP_LIBS_ROS_PUBANDWAIT_NODE_HPP_

