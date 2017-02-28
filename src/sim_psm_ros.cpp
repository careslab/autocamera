// Simulate ECM Node
// Zihan Chen
// 2014-10-15

// standard
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>

#include "mtsSimHWPSM.h"


int main(int argc, char *argv[])
{
    std::cout << "Sim PSM ROS" << std::endl;

    // Cisst Logging Setup
    cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
    cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );
    cmnLogger::AddChannelToStdOut( CMN_LOG_ALLOW_ERRORS_AND_WARNINGS );

    ros::init(argc, argv, "sim_psm_ros");
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    std::string urdfPrefix;
    node_private.getParam("prefix", urdfPrefix);

    // task manager
    mtsManagerLocal* taskManager = mtsManagerLocal::GetInstance();

    // Simulated ECM Hardware
    mtsSimHWPSM hwPSM(&node, "hwPSM", 5 * cmn_ms, urdfPrefix);
    taskManager->AddComponent(&hwPSM);

    // PSM Arm
    mtsIntuitiveResearchKitPSM psm("PSM", 5 * cmn_ms);
    taskManager->AddComponent(&psm);
    std::string psmConfigFile = ros::package::getPath("dvrk_robot");;
    psmConfigFile.append("/config/dvpsm.rob");
    psm.Configure(psmConfigFile);

    // Connect (Required -> Provided)
    taskManager->Connect(psm.GetName(), "PID", hwPSM.GetName(), "PID");
    taskManager->Connect(psm.GetName(), "RobotIO", hwPSM.GetName(), "RobotIO");
    taskManager->Connect(psm.GetName(), "Adapter", hwPSM.GetName(), "Adapter");
    taskManager->Connect(psm.GetName(), "Tool", hwPSM.GetName(), "Tool");
    taskManager->Connect(psm.GetName(), "ManipClutch", hwPSM.GetName(), "ManipClutch");
    taskManager->Connect(psm.GetName(), "SUJClutch", hwPSM.GetName(), "SUJClutch");

    //-------------------------------------------------------
    // Start ROS Bridge
    // ------------------------------------------------------
    // ros wrapper
    mtsROSBridge robotBridge("ROSPSM",     // name
                             5 * cmn_ms,   // period
                             false,        // spin
                             true,         // signal handler
                             &node);       // node handle

    // connect to PSM
    robotBridge.AddPublisherFromReadCommand<prmPositionCartesianGet, geometry_msgs::Pose>(
                "Robot", "GetPositionCartesian", "cartesian_pose_current");
    robotBridge.AddSubscriberToWriteCommand<std::string, std_msgs::String>(
                "Robot", "SetRobotControlState", "set_robot_state");
    robotBridge.AddSubscriberToWriteCommand<prmPositionCartesianSet, geometry_msgs::Pose>(
                "Robot", "SetPositionCartesian", "set_position_cartesian");
    robotBridge.AddSubscriberToWriteCommand<double, std_msgs::Float32>(
                "Robot", "SetOpenAngle", "set_open_angle");

    robotBridge.AddPublisherFromEventWrite<std::string, std_msgs::String>(
                "Robot", "RobotStatusMsg", "robot_status_msg");
    robotBridge.AddPublisherFromEventWrite<std::string, std_msgs::String>(
                "Robot", "RobotErrorMsg", "robot_error_msg");

    // PID
    robotBridge.AddPublisherFromReadCommand<prmPositionJointGet, sensor_msgs::JointState>(
                "PID", "GetPositionJoint", "joint_position_current");
    robotBridge.AddSubscriberToWriteCommand<prmPositionJointSet, sensor_msgs::JointState>(
                "PID", "SetPositionJoint","set_position_joint");

    // Connect
    taskManager->AddComponent(&robotBridge);
    taskManager->Connect(robotBridge.GetName(), "Robot", psm.GetName(), "Robot");
    taskManager->Connect(robotBridge.GetName(), "PID", hwPSM.GetName(), "PID");

    //-------------------------------------------------------
    // End ROS Bridge
    // ------------------------------------------------------

    // -------- Start & Run ---------------
    taskManager->CreateAllAndWait(1.0 * cmn_s);
    taskManager->StartAll();

    ros::spin();

    taskManager->KillAllAndWait(1.0 * cmn_s);
    taskManager->Cleanup();
    cmnLogger::Kill();

    return 0;
}
