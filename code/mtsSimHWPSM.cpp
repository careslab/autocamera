
// Simulate PSM HW
// Zihan Chen
// 2014-10-16

#include <cisstParameterTypes/prmEventButton.h>
#include "mtsSimHWPSM.h"


mtsSimHWPSM::mtsSimHWPSM(ros::NodeHandle *node, const
                         std::string name,
                         const double period,
                         const std::string prefix):
    mtsTaskPeriodic(name, period),
    mNode(node),
    mNumJoints(7),
    mPIDEnabled(false),
    mIsPowerOn(false),
    mAdapterValue(false),
    mToolValue(false)
{
    // Setup ROS Interface
    mSubPSMJntState = mNode->subscribe("joint_states", 1,
                                       &mtsSimHWPSM::CallbackECMJointState, this);
    mPubPSMJntCmd = mNode->advertise<sensor_msgs::JointState>("joint_states_robot", 1);

    mMsgJntCmd.name.clear();
    mMsgJntCmd.name.push_back(prefix + "outer_yaw_joint");
    mMsgJntCmd.name.push_back(prefix + "outer_pitch_joint");
    mMsgJntCmd.name.push_back(prefix + "outer_insertion_joint");
    mMsgJntCmd.name.push_back(prefix + "outer_roll_joint");
    mMsgJntCmd.name.push_back(prefix + "outer_wrist_pitch_joint");
    mMsgJntCmd.name.push_back(prefix + "outer_wrist_yaw_joint");
    mMsgJntCmd.name.push_back(prefix + "outer_wrist_open_angle_joint");

    // Some initialization
    // e.g. joint limits
    mJointUpperLimit.SetSize(mNumJoints);
    mJointLowerLimit.SetSize(mNumJoints);

    // Hard-code Joint Limits
    // -90 to 90 deg
    mJointUpperLimit[0] = cmnPI_2; mJointLowerLimit[0] = -cmnPI_2;    
    // -50 to 50 deg
    mJointUpperLimit[1] = cmnPI * 50.0 / 180.0; mJointLowerLimit[1] = -cmnPI * 50.0 / 180.0;
    // 0 to 0.240 m
    mJointUpperLimit[2] = 0.240; mJointLowerLimit[2] = 0;
    // -130 to 130 deg
    mJointUpperLimit[3] = cmnPI * 130.0 / 180.0; mJointLowerLimit[3] = -cmnPI * 130.0 / 180.0;
    // -90 to 90 deg
    mJointUpperLimit[4] = cmnPI_2; mJointLowerLimit[4] = -cmnPI_2;
    // -80 to 80 deg
    mJointUpperLimit[5] = cmnPI * 80.0 / 180.0; mJointLowerLimit[5] = -cmnPI * 80.0 / 180.0;
    // -10 to 80 deg
    mJointUpperLimit[6] = cmnPI_2; mJointLowerLimit[6] = - cmnPI * 10.0 / 180.0;
    mDesiredPosition.SetSize(mNumJoints);
    mFeedbackPositionParam.SetSize(mNumJoints);

    // Statetable
    StateTable.AddData(mDesiredPosition, "DesiredPosition");
    StateTable.AddData(mCheckJointLimit, "IsCheckJointLimit");
    StateTable.AddData(mFeedbackPositionParam, "prmFeedbackPos");

    // Setup Cisst Interface

    // Simulate PID
    mtsInterfaceProvided* provPID = AddInterfaceProvided("PID");
    if (provPID) {
        provPID->AddCommandWrite(&mtsSimHWPSM::Enable, this, "Enable", mtsBool());
        provPID->AddCommandReadState(StateTable, mFeedbackPositionParam, "GetPositionJoint");
        provPID->AddCommandReadState(StateTable, mDesiredPosition, "GetPositionJointDesired");
        provPID->AddCommandWrite(&mtsSimHWPSM::SetDesiredPositions, this, "SetPositionJoint", mDesiredPositionParam);
        provPID->AddCommandWriteState(StateTable, mCheckJointLimit, "SetCheckJointLimit");
    }

    // Simulate RobotIO
    mtsInterfaceProvided* provIO = AddInterfaceProvided("RobotIO");
    if (provIO) {
        provIO->AddCommandVoid(&mtsSimHWPSM::EnablePower, this, "EnablePower");
        provIO->AddCommandVoid(&mtsSimHWPSM::DisablePower, this, "DisablePower");

        StateTable.AddData(mActuatorPowerStatus, "ActuatorPowerStatus");
        provIO->AddCommandReadState(StateTable, mActuatorPowerStatus,
                                    "GetActuatorAmpStatus"); // vector[bool]
        provIO->AddCommandVoid(&mtsSimHWPSM::BiasEncoder, this, "BiasEncoder");
        provIO->AddCommandWrite(&mtsSimHWPSM::SetActuatorCurrent, this,
                                "SetActuatorCurrent", mActuatorCurrent);
        provIO->AddCommandWrite(&mtsSimHWPSM::SetPotsToEncodersTolerance, this,
                                "SetPotsToEncodersTolerance", mPotsToEncodersTolerance);
        provIO->AddCommandWrite(&mtsSimHWPSM::UsePotsForSafetyCheck, this,
                                "UsePotsForSafetyCheck", mUsePotsForSafetyCheck);
    }

    // Adapter
    mtsInterfaceProvided* provAdapter = AddInterfaceProvided("Adapter");
    if (provAdapter) {
        StateTable.AddData(mAdapterValue, "AdapterValue");
        provAdapter->AddCommandReadState(StateTable, mAdapterValue, "GetButton");
        provAdapter->AddEventWrite(mEventTriggers.Adapter, "Button", prmEventButton());
    }

    // Tool
    mtsInterfaceProvided* provTool = AddInterfaceProvided("Tool");
    if (provTool) {
        StateTable.AddData(mToolValue, "ToolValue");
        provTool->AddCommandReadState(StateTable, mToolValue, "GetButton");
        provTool->AddEventWrite(mEventTriggers.Tool, "Button", prmEventButton());
    }

    // ManipClutch
    mtsInterfaceProvided* provManip = AddInterfaceProvided("ManipClutch");
    if (provManip) {
        provManip->AddEventWrite(mEventTriggers.ManipClutch, "Button", prmEventButton());
    }

    // SUJClutch
    mtsInterfaceProvided* provSUJ = AddInterfaceProvided("SUJClutch");
    if (provSUJ) {
        provSUJ->AddEventWrite(mEventTriggers.SUJClutch, "Button", prmEventButton());
    }
}


void mtsSimHWPSM::Run()
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // What should I do here?
    // Publish Command Position If necessary
}


// ------------------------------------------
// PID
// ------------------------------------------

void mtsSimHWPSM::Enable(const mtsBool & enable)
{
    mPIDEnabled = enable.Data;
}

void mtsSimHWPSM::SetDesiredPositions(const prmPositionJointSet &prmPos)
{
    mDesiredPositionParam = prmPos;
    mDesiredPositionParam.GetGoal(mDesiredPosition);    

    // sanity check
    if (mDesiredPosition.size() != mNumJoints) {
        CMN_LOG_RUN_ERROR << "Desired Position Size Mismatch" << std::endl;
        return;
    }

    mDesiredPosition[2] = mDesiredPosition[2] / 1000.0;  // Convert to SI unit

    // jnt limit check
    if (mCheckJointLimit) {
        // limit check: clip the desired position
        mDesiredPosition.ElementwiseMin(mJointUpperLimit);
        mDesiredPosition.ElementwiseMax(mJointLowerLimit);
    }

    // Publish
    mMsgJntCmd.position.resize(mNumJoints);
    std::copy(mDesiredPosition.begin(), mDesiredPosition.end(), mMsgJntCmd.position.begin());
    mPubPSMJntCmd.publish(mMsgJntCmd);
}


// ------------------------------------------
// RobotIO
// ------------------------------------------
void mtsSimHWPSM::EnablePower(void)
{
    CMN_LOG_RUN_DEBUG << "EnablePower Called" << std::endl;
    mIsPowerOn = true;
}

void mtsSimHWPSM::DisablePower(void)
{
    CMN_LOG_RUN_DEBUG << "DisablePower Called" << std::endl;
    mIsPowerOn = false;
}

void mtsSimHWPSM::BiasEncoder(void)
{
    CMN_LOG_RUN_DEBUG << "BiasEncoder Called" << std::endl;
}

void mtsSimHWPSM::SetActuatorCurrent(const vctDoubleVec & currents)
{
    if (currents.size() != mNumJoints) {
        CMN_LOG_RUN_ERROR << "Wrong currents size" << std::endl;
    }
    mActuatorCurrent.ForceAssign(currents);
}

void mtsSimHWPSM::SetPotsToEncodersTolerance(const vctDoubleVec &tolerance)
{
    if (tolerance.size() != mNumJoints) {
        CMN_LOG_RUN_ERROR << "Wrong tolerance size" << std::endl;
    }
    mPotsToEncodersTolerance.ForceAssign(tolerance);
}

void mtsSimHWPSM::UsePotsForSafetyCheck(const bool &usePotsForSafetyCheck)
{
    mUsePotsForSafetyCheck = usePotsForSafetyCheck;
}

// ------------------------------------------
// ROS
// ------------------------------------------
void mtsSimHWPSM::CallbackECMJointState(const sensor_msgs::JointState &msg)
{
    // sanity check
    vctDoubleVec jntPos(mNumJoints, 0.0);
    if (msg.position.size() != 13) {
        CMN_LOG_RUN_ERROR << "PSM ROS Joint Size Error" << std::endl;
        mFeedbackPositionParam.SetPosition(jntPos);
        return;
    }

    // assign jnt pos
    jntPos[0] = msg.position[0];   // outer_yaw_joint
    jntPos[1] = msg.position[1];   // outer_pitch_joint
    jntPos[2] = msg.position[7] * 1000.0;   // outer_insertion_joint
    jntPos[3] = msg.position[8];   // outer_roll_joint
    jntPos[4] = msg.position[9];   // outer_wrist_pitch_joint
    jntPos[5] = msg.position[10];  // outer_wrist_yaw_joint
    jntPos[6] = msg.position[11];  // outer_wrist_open_angle_joint
    mFeedbackPositionParam.SetPosition(jntPos);
}





