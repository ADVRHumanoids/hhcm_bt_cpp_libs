#include <hhcm_bt_cpp_libs/ObjectComputeGraspingOffset.h>

using hhcm_bt::ObjectComputeGraspingOffset;


ObjectComputeGraspingOffset::ObjectComputeGraspingOffset(
    const std::string& name, const BT::NodeConfig& config,
    hhcm_bt::TF::Ptr tf) :
    BT::SyncActionNode(name, config),
    _tf(tf)
{ 
    auto ref_frame_exp = getInput<std::string>("ref_frame");  
    auto ee_frame_exp = getInput<std::string>("ee_frame");  

    if (!ee_frame_exp) {
        throw BT::RuntimeError("Missing required input port: ee_frame");
    }
    if (!ref_frame_exp) {
        throw BT::RuntimeError("Missing required input port: ref_frame");
    }

    ee_frame = ee_frame_exp.value();
    ref_frame = ref_frame_exp.value();

    if (! _tf->addTf(ref_frame, ee_frame)) {
        throw BT::RuntimeError("add tf returned false");
    }
}

BT::PortsList ObjectComputeGraspingOffset::providedPorts() {
    return {
        BT::InputPort<std::string>("ref_frame", ""),
        BT::InputPort<std::string>("ee_frame", ""),
        BT::InputPort<geometry_msgs::Pose>("obj_pose"),
        BT::InputPort<double>("obj_size_x", ""),
        BT::InputPort<double>("obj_size_y", ""),
        BT::InputPort<double>("obj_size_z", ""),
        BT::InputPort<double>("offset_margin", 0.04, ""),
        BT::OutputPort<double>("offset")
    };
}

BT::NodeStatus ObjectComputeGraspingOffset::tick() {
    
    auto obj_pose_exp = getInput<geometry_msgs::Pose>("obj_pose");
    auto obj_size_x_exp = getInput<double>("obj_size_x");
    auto obj_size_y_exp = getInput<double>("obj_size_y");
    auto obj_size_z_exp = getInput<double>("obj_size_z");
    auto offset_margin_exp = getInput<double>("offset_margin");

    if (!obj_pose_exp) {
        throw BT::RuntimeError("Missing required input port: obj_pose");
    }
    if (!obj_size_x_exp) {
        throw BT::RuntimeError("Missing required input port: obj_size_x");
    }
    if (!obj_size_y_exp) {
        throw BT::RuntimeError("Missing required input port: obj_size_y");
    }
    if (!obj_size_z_exp) {
        throw BT::RuntimeError("Missing required input port: obj_size_z");
    }

    geometry_msgs::Pose obj_pose = obj_pose_exp.value();
    double obj_size_x = obj_size_x_exp.value();
    double obj_size_y = obj_size_y_exp.value();
    double obj_size_z = obj_size_z_exp.value();

    tf2::Stamped<tf2::Transform> ref_T_ee;
    if(! _tf->getTf(std::make_pair(ref_frame, ee_frame), ref_T_ee, 1)) {
        ROS_ERROR_STREAM("ObjectComputeGraspingOffset::tick ERROR: tf not found");
        return BT::NodeStatus::FAILURE;
    }

    double z = ref_T_ee.getOrigin().getZ() - obj_pose.position.z + obj_size_z/2 + offset_margin_exp.value();

    setOutput("offset", z);

    return BT::NodeStatus::SUCCESS;
}