#include <hhcm_bt_cpp_libs/CheckStillTransform.h>

using hhcm_bt::CheckStillTransform;

CheckStillTransform::CheckStillTransform(
    const std::string& name, const BT::NodeConfig& config,
    hhcm_bt::TF::Ptr tf) :
    BT::ConditionNode(name, config),
    _tf(tf)
{ 

    auto ref_frame_exp = getInput<std::string>("ref_frame"); 
    auto frame_exp = getInput<std::string>("frame"); 
    auto still_threshold_pos_exp = getInput<double>("still_threshold_pos");
    auto still_threshold_sec_exp = getInput<double>("still_threshold_sec");

    if (!ref_frame_exp) {

        throw BT::RuntimeError("ref_frame port is empty!");
    }    
    if (!frame_exp) {

        throw BT::RuntimeError("frame port is empty!");
    }    
    if (!still_threshold_pos_exp) {

        throw BT::RuntimeError("still_threshold_pos port is empty!");
    }    
    if (!still_threshold_sec_exp) {

        throw BT::RuntimeError("still_threshold_pos port is empty!");
    }

    ref_frame = ref_frame_exp.value();
    frame = frame_exp.value();
    still_threshold_pos = still_threshold_pos_exp.value();
    still_threshold_sec = still_threshold_sec_exp.value();
    
    if (! _tf->addTf(ref_frame, frame)) {
        throw BT::RuntimeError("add tf returned false");
    }

    still_time = ros::Duration(0);

}


BT::PortsList CheckStillTransform::providedPorts() {
    
    return {BT::InputPort<std::string>("ref_frame"),
            BT::InputPort<std::string>("frame"),
            BT::InputPort<double>("still_threshold_pos"),
            BT::InputPort<double>("still_threshold_sec"),
            BT::OutputPort<double>("still_sec")
    };
}

BT::NodeStatus CheckStillTransform::tick() { 

    if ( (_tf->getTf(std::make_pair(ref_frame, frame), tf_now, 1)) && 
         (std::sqrt (
            std::pow(tf_now.getOrigin().getX() - tf_prev.getOrigin().getX(), 2) +
            std::pow(tf_now.getOrigin().getY() - tf_prev.getOrigin().getY(), 2) 
            + std::pow(tf_now.getOrigin().getZ() - tf_prev.getOrigin().getZ(), 2) 
            ) < still_threshold_pos ) && 
         (std::sqrt (
            std::pow(tf_now.getOrigin().getX() - tf_zero.getOrigin().getX(), 2) +
            std::pow(tf_now.getOrigin().getY() - tf_zero.getOrigin().getY(), 2) 
            + std::pow(tf_now.getOrigin().getZ() - tf_zero.getOrigin().getZ(), 2) 
            ) < still_threshold_pos )
        ) 
    {
        still_time = tf_now.stamp_ - tf_zero.stamp_;

    } else {
        still_time = ros::Duration(0);
        tf_zero = tf_now;
    }
    
    setOutput("still_sec", still_time.toSec());
    tf_prev = tf_now;

    if (still_time.toSec() > still_threshold_sec) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}
