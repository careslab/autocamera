#ifndef _mtsSimHWPSM_h
#define _mtsSimHWPSM_h

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

class mtsSimHWPSM : public mtsTaskPeriodic {

private:
  // ROS
  ros::NodeHandle* mNode;
  ros::Subscriber mSubPSMJntState;
  ros::Publisher mPubPSMJntCmd;
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
  bool mAdapterValue;
  bool mToolValue;
  vctBoolVec mActuatorPowerStatus;
  vctBoolVec mBrakePowerStatus;
  vctDoubleVec mActuatorCurrent;
  vctDoubleVec mPotsToEncodersTolerance;
  bool mUsePotsForSafetyCheck;

  // Clutch Events
  struct {
      mtsFunctionWrite Adapter;
      mtsFunctionWrite Tool;
      mtsFunctionWrite ManipClutch;
      mtsFunctionWrite SUJClutch;
  } mEventTriggers;

public:
  mtsSimHWPSM(ros::NodeHandle* node,
              const std::string name,
              const double period = 10 * cmn_ms,
              const std::string prefix = "one_");
  ~mtsSimHWPSM(){}

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
  void SetPotsToEncodersTolerance(const vctDoubleVec & tolerance);
  void UsePotsForSafetyCheck(const bool & usePotsForSafetyCheck);

  // ----------------------------------
  // ROS Callback
  // ----------------------------------
  // From Robot
  void CallbackECMJointState(const sensor_msgs::JointState &msg);
};

#endif   // _mtsSimHWPSM_h

