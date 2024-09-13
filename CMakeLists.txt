cmake_minimum_required(VERSION 3.10.2)
project(hhcm_bt_cpp_libs)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
    roscpp
    rospy
    geometry_msgs
    control_msgs
    tf2_ros
    tf2_msgs
    xbot_msgs
    tpo_msgs
    franka_gripper
    moveit_msgs
    moveit_core
)

find_package(behaviortree_cpp REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
    ${PROJECT_NAME}_TF 
    ${PROJECT_NAME}_TFHandler 
    ${PROJECT_NAME}_TFReaderNode 
    ${PROJECT_NAME}_GripperCommandRosActionNode 
    ${PROJECT_NAME}_FrankaGripperGraspRosActionNode 
    ${PROJECT_NAME}_SetStringRosClientNode
    ${PROJECT_NAME}_SetStringRosServerHandler
    ${PROJECT_NAME}_TwistRosPubNode 
    ${PROJECT_NAME}_TwistRosContinuosPubNode 
    ${PROJECT_NAME}_CheckStillTransform 
    ${PROJECT_NAME}_CheckClosestFrame
    ${PROJECT_NAME}_CheckClosestFramePub
    ${PROJECT_NAME}_StringRosSubHandler
    ${PROJECT_NAME}_TpoPosesRosActionNode 
    ${PROJECT_NAME}_ObjectPlanningSceneRosClientNode
    ${PROJECT_NAME}_ObjectPlanningSceneRosPubNode
    ${PROJECT_NAME}_ObjectPlanningSceneRosPubCliNode
  CATKIN_DEPENDS 
    roscpp
    rospy
    geometry_msgs
    control_msgs
    tf2_ros
    tf2_msgs
    xbot_msgs
    tpo_msgs
    moveit_msgs
    moveit_core
    franka_gripper
  DEPENDS 
    behaviortree_cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_TF
    src/TF.cpp
)
target_link_libraries(${PROJECT_NAME}_TF
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_TF ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_TFHandler
    src/TFHandler.cpp
)
target_link_libraries(${PROJECT_NAME}_TFHandler 
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_TF
)
add_dependencies(${PROJECT_NAME}_TFHandler ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_TFReaderNode
    src/TFReaderNode.cpp
)
target_link_libraries(${PROJECT_NAME}_TFReaderNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
    ${PROJECT_NAME}_TF
)
add_dependencies(${PROJECT_NAME}_TFReaderNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_GripperCommandRosActionNode
    src/GripperCommandRosActionNode.cpp
)
target_link_libraries(${PROJECT_NAME}_GripperCommandRosActionNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_GripperCommandRosActionNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_FrankaGripperGraspRosActionNode
    src/FrankaGripperGraspRosActionNode.cpp
)
target_link_libraries(${PROJECT_NAME}_FrankaGripperGraspRosActionNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_FrankaGripperGraspRosActionNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_SetStringRosClientNode
    src/SetStringRosClientNode.cpp
)
target_link_libraries(${PROJECT_NAME}_SetStringRosClientNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_SetStringRosClientNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_SetStringRosServerHandler
    src/SetStringRosServerHandler.cpp
)
target_link_libraries(${PROJECT_NAME}_SetStringRosServerHandler 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_SetStringRosServerHandler ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_TwistRosContinuosPubNode
    src/TwistRosContinuosPubNode.cpp
)
target_link_libraries(${PROJECT_NAME}_TwistRosContinuosPubNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_TwistRosContinuosPubNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_TwistRosPubNode
    src/TwistRosPubNode.cpp
)
target_link_libraries(${PROJECT_NAME}_TwistRosPubNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_TwistRosPubNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_CheckStillTransform
    src/CheckStillTransform.cpp
)
target_link_libraries(${PROJECT_NAME}_CheckStillTransform 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_CheckStillTransform ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_CheckClosestFrame
    src/CheckClosestFrame.cpp
)
target_link_libraries(${PROJECT_NAME}_CheckClosestFrame 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_CheckClosestFrame ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_CheckClosestFramePub
    src/CheckClosestFramePub.cpp
)
target_link_libraries(${PROJECT_NAME}_CheckClosestFramePub 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_CheckClosestFramePub ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_StringRosSubHandler
    src/StringRosSubHandler.cpp
)
target_link_libraries(${PROJECT_NAME}_StringRosSubHandler 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_StringRosSubHandler ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_TpoPosesRosActionNode
    src/TpoPosesRosActionNode.cpp
)
target_link_libraries(${PROJECT_NAME}_TpoPosesRosActionNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_TpoPosesRosActionNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_ObjectPlanningSceneRosClientNode
    src/ObjectPlanningSceneRosClientNode.cpp
)
target_link_libraries(${PROJECT_NAME}_ObjectPlanningSceneRosClientNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_ObjectPlanningSceneRosClientNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_ObjectPlanningSceneRosPubNode
    src/ObjectPlanningSceneRosPubNode.cpp
)
target_link_libraries(${PROJECT_NAME}_ObjectPlanningSceneRosPubNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_ObjectPlanningSceneRosPubNode ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_ObjectPlanningSceneRosPubCliNode
    src/ObjectPlanningSceneRosPubCliNode.cpp
)
target_link_libraries(${PROJECT_NAME}_ObjectPlanningSceneRosPubCliNode 
    ${catkin_LIBRARIES}
    BT::behaviortree_cpp
)
add_dependencies(${PROJECT_NAME}_ObjectPlanningSceneRosPubCliNode ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(
    TARGETS 
        ${PROJECT_NAME}_TF 
        ${PROJECT_NAME}_TFHandler 
        ${PROJECT_NAME}_TFReaderNode 
        ${PROJECT_NAME}_GripperCommandRosActionNode 
        ${PROJECT_NAME}_FrankaGripperGraspRosActionNode 
        ${PROJECT_NAME}_SetStringRosClientNode
        ${PROJECT_NAME}_SetStringRosServerHandler
        ${PROJECT_NAME}_TwistRosContinuosPubNode 
        ${PROJECT_NAME}_TwistRosPubNode
        ${PROJECT_NAME}_CheckStillTransform 
        ${PROJECT_NAME}_CheckClosestFrame
        ${PROJECT_NAME}_CheckClosestFramePub
        ${PROJECT_NAME}_StringRosSubHandler
        ${PROJECT_NAME}_TpoPosesRosActionNode 
        ${PROJECT_NAME}_ObjectPlanningSceneRosClientNode
        ${PROJECT_NAME}_ObjectPlanningSceneRosPubNode
        ${PROJECT_NAME}_ObjectPlanningSceneRosPubCliNode
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tpo_bt.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)