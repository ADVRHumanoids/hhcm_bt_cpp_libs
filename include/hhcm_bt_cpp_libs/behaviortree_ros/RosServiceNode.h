// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// taken from https://github.com/BehaviorTree/BehaviorTree.ROS,
// but using the class StatefulActionNode and not directly the ActionNodeBase

#ifndef HHCM_BT_CPP_LIBS_ROS_SERVICE_NODE_HPP_
#define HHCM_BT_CPP_LIBS_ROS_SERVICE_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>
#include <ros/service_client.h>

namespace BT
{

/**
 * Base Action to implement a ROS Service
 */
template<class ServiceT>
class RosServiceNode : public BT::StatefulActionNode
{
protected:

    RosServiceNode(const std::string& name, const BT::NodeConfig & conf, 
    ros::NodeHandle* nh, const std::string& service_name):
    BT::StatefulActionNode(name, conf), node_(nh) {
        
        service_client_ = node_->serviceClient<ServiceT>( service_name );
    }

public:

    using BaseClass    = RosServiceNode<ServiceT>;
    using ServiceType  = ServiceT;
    using RequestType  = typename ServiceT::Request;
    using ResponseType = typename ServiceT::Response;

    RosServiceNode() = delete;

    virtual ~RosServiceNode() = default;

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

    /// User must implement this method.
    virtual bool prepareRequest(RequestType& request) = 0;

    // OPTIONAL, called at each tick
    virtual bool modifyRequest(RequestType& request) {return true;};

    /// Method (to be implemented by the user) to receive the reply.
    /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
    virtual NodeStatus onResponse( const ResponseType& res)
    {
        return BT::NodeStatus::SUCCESS;
    };

    enum FailureCause{
        MISSING_SERVER = 0,
        FAILED_CALL = 1,
    };

    /// Called when a service call failed. Can be overriden by the user.
    virtual NodeStatus onFailedRequest(FailureCause failure)
    {
        printf("Fail with failure : %d\n", failure);
        return NodeStatus::FAILURE;
    }

protected:

    ros::ServiceClient service_client_;

    RequestType request_;
    ResponseType response_;

    // The node that will be used for any ROS operations
    ros::NodeHandle* node_;
    
    virtual BT::NodeStatus onStart() override {
        
        unsigned msec = getInput<unsigned>("timeout").value();
        ros::Duration timeout(static_cast<double>(msec) * 1e-3);
        
        bool connected = service_client_.waitForExistence(timeout);
        if( !connected ){
            return onFailedRequest(MISSING_SERVER);
        }
        
        if (!prepareRequest(request_)) {
            return NodeStatus::FAILURE;
        }

        
        return NodeStatus::RUNNING;
    }


    BT::NodeStatus onRunning() override
    {

        if (!modifyRequest(request_)) {
            return NodeStatus::FAILURE;
        }

        bool received = service_client_.call( request_, response_ );
        if( !received )
        {
            return onFailedRequest(FAILED_CALL);
        }
        return onResponse(response_);
    }
    
    virtual void onHalted() override {
    }

};


}  // namespace BT

#endif  // HHCM_BT_CPP_LIBS_ROS_SERVICE_NODE_HPP_
