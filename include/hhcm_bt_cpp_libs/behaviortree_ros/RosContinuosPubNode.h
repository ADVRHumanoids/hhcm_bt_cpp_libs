// taken from https://github.com/BehaviorTree/BehaviorTree.ROS and ROS2 version,
// but using the class StatefulActionNode and not directly the ActionNodeBase

#ifndef HHCM_BT_CPP_LIBS_ROS_CONTINUOS_PUB_NODE_HPP_
#define HHCM_BT_CPP_LIBS_ROS_CONTINUOS_PUB_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

template<typename PubMsgType>
class RosContinuosPubNode : public BT::StatefulActionNode
{
protected:
    
    RosContinuosPubNode(const std::string& name, const BT::NodeConfig & conf, ros::NodeHandle* nh,
                  const std::string& pub_topic_name):
        BT::StatefulActionNode(name, conf), node_(nh)
    {
        pub_ = node_->advertise<PubMsgType>(pub_topic_name, 1);
    }
    
    PubMsgType msg_;    

public:

    RosContinuosPubNode() = delete;

    virtual ~RosContinuosPubNode() = default;
    
    // aliases just for the RegisterRosPub at the bottom
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
        PortsList basic = {};
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

    /**
     * called by the onStart, when trasitioning from idle
     * OPTIONAL override, as default it does nothing.
     */
    virtual bool onStartInitialization() {
        return true;
    }

    /**
     * This method is called at each tick, and can be used to modify the msg and publish a different one 
     * at each tick.
     * OPTIONAL override, as default it does nothing.
     */
    virtual bool modifyMsg() {
        return true;
    }

    /// Method called once, when transitioning from the state IDLE.
    /// If it returns RUNNING, this becomes an asynchronous node.
    virtual NodeStatus onStart() override final {

        if (! onStartInitialization()) {
            return BT::NodeStatus::FAILURE;
        } 
        return BT::NodeStatus::RUNNING;
    }

    /// method invoked when the action is already in the RUNNING state.
    virtual NodeStatus onRunning() final {
        if (! modifyMsg()) {
            return BT::NodeStatus::FAILURE;
        }
        pub_.publish(msg_);
        return BT::NodeStatus::RUNNING;
    }

    /// when the method halt() is called and the action is RUNNING, this method is invoked.
    /// This is a convenient place todo a cleanup, if needed.
    virtual void onHalted() override final{
        
    }

private:

    ros::Publisher pub_;
    
    ros::NodeHandle* node_;
      
};

        
}  // namespace BT

#endif  // HHCM_BT_CPP_LIBS_ROS_CONTINUOS_PUB_NODE_HPP_

