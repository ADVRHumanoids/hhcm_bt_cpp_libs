#ifndef HHCM_BT_CPP_LIBS_CHECK_CLOSEST_FRAME_H
#define HHCM_BT_CPP_LIBS_CHECK_CLOSEST_FRAME_H

#include <ros/ros.h>
#include <behaviortree_cpp/condition_node.h>
#include <hhcm_bt_cpp_libs/TF.h>

namespace hhcm_bt {

class CheckClosestFrame : public BT::ConditionNode {

public:
    CheckClosestFrame(const std::string& name, const BT::NodeConfig& config, hhcm_bt::TF::Ptr tf);
    virtual ~CheckClosestFrame() override = default;

    virtual BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    hhcm_bt::TF::Ptr _tf;
    tf2::Stamped<tf2::Transform> _transform;

    std::string _tracking_frame;
    std::vector<std::string> _check_frames;
    std::vector<double> _threshold;

    std::string _closest_frame;


};

} //namespace

#endif //HHCM_BT_CPP_LIBS_CHECK_CLOSEST_FRAME_H