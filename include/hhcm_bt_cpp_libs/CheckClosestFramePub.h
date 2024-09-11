#ifndef HHCM_BT_CPP_LIBS_CHECK_CLOSEST_FRAME_PUB_H
#define HHCM_BT_CPP_LIBS_CHECK_CLOSEST_FRAME_PUB_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hhcm_bt_cpp_libs/behaviortree_ros/RosPubNode.h>
#include <hhcm_bt_cpp_libs/TF.h>
#include <hhcm_bt_cpp_libs/PortGeometryMsgsConversions.h>

namespace hhcm_bt {

class CheckClosestFramePub : public BT::RosPubNode<std_msgs::String> {

public:
    CheckClosestFramePub(const std::string& name, const BT::NodeConfig& config, 
        ros::NodeHandle* nh, const std::string& pub_topic_name, hhcm_bt::TF::Ptr tf);
    virtual ~CheckClosestFramePub() override = default;

    virtual bool modifyMsg() override;

    static BT::PortsList providedPorts();

private:
    hhcm_bt::TF::Ptr _tf;
    tf2::Stamped<tf2::Transform> _transform;

    std::string _tracking_frame;
    std::vector<std::string> _check_frames;
    geometry_msgs::Vector3 _threshold;

    std::string _closest_frame;


};

} //namespace

#endif //HHCM_BT_CPP_LIBS_CHECK_CLOSEST_FRAME_PUB_H