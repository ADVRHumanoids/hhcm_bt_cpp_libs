#include <hhcm_bt_cpp_libs/CheckClosestFrame.h>

using hhcm_bt::CheckClosestFrame;

CheckClosestFrame::CheckClosestFrame(
    const std::string& name, const BT::NodeConfig& config, hhcm_bt::TF::Ptr tf) :
    BT::ConditionNode(name, config),
    _tf(tf)
{

    auto tracking_frame_exp = getInput<std::string>("tracking_frame");
    auto check_frames_exp = getInput<std::vector<std::string>>("check_frames");
    auto threshold_exp = getInput<geometry_msgs::Vector3>("threshold");

    if (!tracking_frame_exp || tracking_frame_exp.value().size() == 0) {
        throw BT::RuntimeError("tracking_frame port not defined or empty!");
    }    
    if (!check_frames_exp || check_frames_exp.value().size() == 0) {
        throw BT::RuntimeError("check_frames port not defined or it is an empty vector!");
    }
    if (!threshold_exp) {
        std::cout << "WARN threshold port empty or not valid: using default threshold 0.01, 0.01, 0.005";
        _threshold.x = 0.01;
        _threshold.y = 0.01;
        _threshold.z = 0.005;

    } else {
        _threshold = threshold_exp.value();
    }

    _tracking_frame = tracking_frame_exp.value();
    _check_frames = check_frames_exp.value();

    for (const auto& check_frame : _check_frames) {

        if (! _tf->addTf(_tracking_frame, check_frame)) {
            throw BT::RuntimeError("add tf returned false");
        }
    }
}

BT::PortsList CheckClosestFrame::providedPorts() {

    return {
        BT::InputPort<std::string>("tracking_frame"),
        BT::InputPort<std::vector<std::string>>("check_frames"),
        BT::InputPort<geometry_msgs::Vector3>("threshold"),
        BT::OutputPort<std::string>("closest_frame")
    };
}


BT::NodeStatus CheckClosestFrame::tick() {

    _closest_frame = "none";

    for (size_t i = 0; i < _check_frames.size(); ++i)
    {
        if (_tf->getTf(std::make_pair(_tracking_frame, _check_frames[i]), _transform, 1)) {
        
            if (std::abs(_transform.getOrigin().getX()) < _threshold.x &&
                std::abs(_transform.getOrigin().getY()) < _threshold.y &&
                std::abs(_transform.getOrigin().getZ()) < _threshold.z )
            {
                _closest_frame = _check_frames[i];
                break;
            }
        }
    }
    
    setOutput("closest_frame", _closest_frame);

    if (_closest_frame.compare("none") == 0) {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}
