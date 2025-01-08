// taken from https://github.com/BehaviorTree/BehaviorTree.ROS,
// but using the class StatefulActionNode and not directly the ActionNodeBase

#ifndef HHCM_BT_CPP_LIBS_ROS_ACTION_NODE_HPP_
#define HHCM_BT_CPP_LIBS_ROS_ACTION_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>  

namespace BT
{

template<class ActionT>
class RosActionNode : public BT::StatefulActionNode
{
public:
    
    RosActionNode(const std::string& name, const BT::NodeConfig& config, ros::NodeHandle* nh, const std::string& server_name):
        BT::StatefulActionNode(name, config), node_(nh), server_name_(server_name) 
    {
        action_client_ = std::make_shared<ActionClientType>( *node_, server_name_, true );
    }

public:

    using BaseClass  = RosActionNode<ActionT>;
    using ActionClientType = actionlib::SimpleActionClient<ActionT>;
    using ActionType = ActionT;
    using GoalType   = typename ActionT::_action_goal_type::_goal_type;
    using ResultType = typename ActionT::_action_result_type::_result_type;

//     RosActionNode() = delete;

    virtual ~RosActionNode() = default;

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
            InputPort<unsigned>("timeout", 100, "timeout to connect to server (milliseconds)")
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

    /// Method called when the Action makes a transition from IDLE to RUNNING.
    /// If it return false, the entire action is immediately aborted, it returns
    /// FAILURE and no request is sent to the server.
    virtual bool prepareGoal(GoalType& goal) = 0;

    /// Method (to be implemented by the user) to receive the reply.
    /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
    virtual NodeStatus onResult( const ResultType& res) 
    {
        return BT::NodeStatus::SUCCESS;
    }

    enum FailureCause{
        MISSING_SERVER = 0,
        ABORTED_BY_SERVER = 1,
        REJECTED_BY_SERVER = 2,
        PREEMPTED_BY_SERVER = 2,
        PREPARE_GOAL_FAIL = 3,
    };

    /// Called when a service call failed. Can be overriden by the user.
    virtual NodeStatus onFailedRequest(FailureCause failure)
    {
        if (failure == MISSING_SERVER){
            ROS_ERROR_NAMED(name(), "ROS Action client received failure from server with failure MISSING_SERVER : %s server not found\n", (server_name_).c_str());

        } else {
            ROS_ERROR_NAMED(name(), "ROS Action client received failure from server with failure : %d\n", failure);
            
        }
        return NodeStatus::FAILURE;
    }
  
    virtual BT::NodeStatus onStart() override
    {
        unsigned msec = getInput<unsigned>("timeout").value();
        msec = (msec >= 500) ? msec : 500;
        ros::Duration timeout(static_cast<double>(msec) * 1e-3);

        if( !action_client_->waitForServer(timeout) ){
            return onFailedRequest(MISSING_SERVER);
        }
        
        GoalType goal;
        if( !prepareGoal(goal) )
        {
            return onFailedRequest(PREPARE_GOAL_FAIL);
        }
        action_client_->sendGoal(goal);
        
        return NodeStatus::RUNNING;
    }
    
    virtual BT::NodeStatus onRunning() override
    {
        // RUNNING
        auto action_state = action_client_->getState();

        // Please refer to these states

        if( action_state == actionlib::SimpleClientGoalState::PENDING ||
            action_state == actionlib::SimpleClientGoalState::ACTIVE )
        {
            return NodeStatus::RUNNING;
        }
        
        else if( action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            return onResult( *action_client_->getResult() );
        }
        
        else if( action_state == actionlib::SimpleClientGoalState::ABORTED)
        {
            return onFailedRequest( ABORTED_BY_SERVER );
        }
        
        else if( action_state == actionlib::SimpleClientGoalState::REJECTED)
        {
            return onFailedRequest( REJECTED_BY_SERVER );
        }
        
        else if( action_state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            return onFailedRequest( PREEMPTED_BY_SERVER );
        }
        
        else
        {
        // FIXME: is there any other valid state we should consider?
            throw std::logic_error("Unexpected state in RosActionNode::tick() : " + action_state.toString());
        }
    }
    
    virtual void onHalted() override
    {
        action_client_->cancelAllGoals();
    }
  
protected:

  std::shared_ptr<ActionClientType> action_client_;

  ros::NodeHandle* node_;
  const std::string server_name_;
};

}  // namespace BT

#endif  // HHCM_BT_CPP_LIBS_ROS_ACTION_NODE_HPP_

