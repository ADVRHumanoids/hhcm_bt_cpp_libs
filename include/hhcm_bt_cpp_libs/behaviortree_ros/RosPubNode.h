// taken from https://github.com/BehaviorTree/BehaviorTree.ROS,

#ifndef HHCM_BT_CPP_LIBS_ROS_PUB_NODE_HPP_
#define HHCM_BT_CPP_LIBS_ROS_PUB_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

template<typename PubMsgType>
class RosPubNode : public BT::SyncActionNode
{
protected:
    
    RosPubNode(const std::string& name, const BT::NodeConfig & conf, ros::NodeHandle* nh,
                  const std::string& pub_topic_name):
        BT::SyncActionNode(name, conf), node_(nh)
    {
        pub_ = node_->advertise<PubMsgType>(pub_topic_name, 1);
    }
    
    PubMsgType msg_;    

public:

    RosPubNode() = delete;

    virtual ~RosPubNode() = default;
    
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
     * This method is called at each tick, and can be used to modify the msg and publish a different one 
     * at each tick.
     * OPTIONAL override, as default it does nothing.
     */
    virtual bool modifyMsg() {
        return true;
    }

    BT::NodeStatus tick() override final {

        if (! modifyMsg()) {
            return BT::NodeStatus::FAILURE;
        }
        pub_.publish(msg_);
        return BT::NodeStatus::SUCCESS;
    }

protected:

    ros::Publisher pub_;
private:

    ros::NodeHandle* node_;
      
};
        
}  // namespace BT

#endif  // HHCM_BT_CPP_LIBS_ROS_PUB_NODE_HPP_

