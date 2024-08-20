#ifndef HHCM_BT_CPP_CHECK_STILL_TRANSFORM_H
#define HHCM_BT_CPP_CHECK_STILL_TRANSFORM_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

#include <behaviortree_cpp/condition_node.h>

#include <hhcm_bt_cpp_libs/TF.h>
#include <hhcm_bt_cpp_libs/PortRosTimeConversions.h>


/**
 * 
 */

namespace hhcm_bt {

class CheckStillTransform : public BT::ConditionNode {

public: 
    CheckStillTransform(const std::string& name, const BT::NodeConfig& config, hhcm_bt::TF::Ptr tf);
    virtual ~CheckStillTransform() override = default;

    virtual BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    hhcm_bt::TF::Ptr _tf;
    tf2::Stamped<tf2::Transform> tf_now, tf_prev, tf_zero;
    std::string ref_frame, frame;
    ros::Duration still_time;
    double still_threshold_pos, still_threshold_sec;

};

} //namespace

#endif //HHCM_BT_CPP_CHECK_STILL_TRANSFORM_H
    