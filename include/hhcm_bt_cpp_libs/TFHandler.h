#ifndef HHCM_BT_CPP_LIBS_TF_HANDLER_H
#define HHCM_BT_CPP_LIBS_TF_HANDLER_H

#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

/**
 * This class is intended to have a unique tf listener
 * that fills the requested tf at each loop. The handler
 * is created in the main, outside the tree. Each node that 
 * necessitate a tf, will have the TF::Ptr as constructor 
 * argument, such that to read the necessary tf, that can be
 * added with addTf in the initialization of the node.
 * The handler will check every added tf and find the relative
 * transforms.
 * 
 */

namespace hhcm_bt {

struct TF {
    
    typedef std::shared_ptr<TF> Ptr;
    typedef std::shared_ptr<const TF> ConstPtr;
    
    TF() {};
    
    std::map<std::pair<std::string, std::string>, tf2::Stamped<tf2::Transform> > x_T_x;
    
    bool addTf(const std::pair<std::string, std::string>& key);
    bool addTf(const std::string& from, const std::string& to);

};
    
class TFHandler {
    
public:
    TFHandler(ros::NodeHandle* nh);
    
    bool getTf();
    
    bool waitForRos(const std::string &from, const std::string &to);
    
    TF::Ptr tf;

  
private:
    
    ros::NodeHandle* nh;
    TF::Ptr tf_internal;
    
    ros::Time last_time;
    
    tf2_ros::Buffer tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  
};
    
} //namespace

#endif //HHCM_BT_CPP_LIBS_TF_HANDLER_H
