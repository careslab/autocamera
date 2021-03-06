#ifndef _mtsSimHWECM_h
#define _mtsSimHWECM_h

// Cisst
#include <iostream>
#include <cisstCommon.h>
#include <cisstVector.h>
#include <cisstMultiTask.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

class mtsSimHWECM : public mtsTaskPeriodic {

private:
  // ROS
  ros::NodeHandle* mNode;
  ros::Subscriber mSubJntState;
  ros::Subscriber mSubManipClutch;
  ros::Subscriber mSubSUJClutch;
  ros::Publisher mPubECMJntCmd;
  sensor_msgs::JointState mMsgJntCmd;

  // Common
  size_t mNumJoints;

  // PID State
  bool mPIDEnabled;
  bool mCheckJointLimit;
  vctDoubleVec mDesiredPosition;
  prmPositionJointGet mFeedbackPositionParam;
  prmPositionJointSet mDesiredPositionParam;

  vctDoubleVec mJointUpperLimit;
  vctDoubleVec mJointLowerLimit;

  // RobotIO State
  bool mIsPowerOn;
  bool mIsBrakeReleased;
  vctBoolVec mActuatorPowerStatus;
  vctBoolVec mBrakePowerStatus;
  vctDoubleVec mActuatorCurrent;

  // Clutch Events
  mtsFunctionWrite mManipClutchEventTrigger;
  mtsFunctionWrite mSUJClutchEventTrigger;

public:
  mtsSimHWECM(ros::NodeHandle* node,
                  const std::string name,
                  const double period = 10 * cmn_ms);
  ~mtsSimHWECM(){}

  void Configure( const std::string& name=""){}
  void Startup(){}
  void Run();
  void Cleanup(){}

private:
  void SetMasterControlState(void);

  // PID
  void Enable(const mtsBool & enable);
  void SetDesiredPositions(const prmPositionJointSet & prmPos);

  // RobotIO
  void EnablePower(void);
  void DisablePower(void);

  void BiasEncoder(void);
  void SetActuatorCurrent(const vctDoubleVec & currents);

  void BrakeRelease(void);
  void BrakeEngage(void);

  // ----------------------------------
  // ROS Callback
  // ----------------------------------
  // From Robot
  void CallbackECMJointState(const sensor_msgs::JointState &msg);
  void CallbackECMManipClutch(const std_msgs::Bool &msg);
  void CallbackECMSUJClutch(const std_msgs::Bool &msg);
};

#endif   // _mtsSimHWECM_h

