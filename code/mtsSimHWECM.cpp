
// Simulate ECM HW
// Zihan Chen
// 2014-10-15

#include <cisstParameterTypes/prmEventButton.h>
#include "mtsSimHWECM.h"


mtsSimHWECM::mtsSimHWECM(ros::NodeHandle *node, const
                         std::string name,
                         const double period):
    mtsTaskPeriodic(name, period),
    mNode(node),
    mNumJoints(4),
    mPIDEnabled(false),
    mIsPowerOn(false),
    mIsBrakeReleased(false)
{
    // Setup ROS Interface
    mSubJntState = mNode->subscribe("/dvrk_ecm/joint_states", 1,
                                       &mtsSimHWECM::CallbackECMJointState, this);
    mSubManipClutch = mNode->subscribe("/dvrk_ecm/manip_clutch", 1,
                                       &mtsSimHWECM::CallbackECMManipClutch, this);
    mSubSUJClutch = mNode->subscribe("/dvrk_ecm/suj_clutch", 1,
                                       &mtsSimHWECM::CallbackECMSUJClutch, this);

    mPubECMJntCmd = mNode->advertise<sensor_msgs::JointState>("/dvrk_ecm/joint_states_robot", 1);

    mMsgJntCmd.name.clear();
    mMsgJntCmd.name.push_back("ecm_yaw_joint");
    mMsgJntCmd.name.push_back("ecm_pitch_joint");
    mMsgJntCmd.name.push_back("ecm_insertion_joint");
    mMsgJntCmd.name.push_back("ecm_roll_joint");


    // Some initialization
    // e.g. joint limits
    mJointUpperLimit.SetSize(mNumJoints);
    mJointLowerLimit.SetSize(mNumJoints);

    // Hard-code Joint Limits
    mJointUpperLimit[0] = cmnPI_2; mJointLowerLimit[0] = -cmnPI_2;
    mJointUpperLimit[1] = cmnPI * 65.0 / 180.0; mJointLowerLimit = -cmnPI * 45.0 / 180.0;
    mJointUpperLimit[2] = 0.235; mJointLowerLimit[2] = 0;
    mJointUpperLimit[3] = cmnPI_2; mJointLowerLimit[3] = -cmnPI_2;

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
        provPID->AddCommandWrite(&mtsSimHWECM::Enable, this, "Enable", mtsBool());
        provPID->AddCommandReadState(StateTable, mFeedbackPositionParam, "GetPositionJoint");
        provPID->AddCommandReadState(StateTable, mDesiredPosition, "GetPositionJointDesired");

        provPID->AddCommandWrite(&mtsSimHWECM::SetDesiredPositions, this, "SetPositionJoint", mDesiredPositionParam);
        provPID->AddCommandWriteState(StateTable, mCheckJointLimit, "SetCheckJointLimit");
    }

    // Simulate RobotIO
    mtsInterfaceProvided* provIO = AddInterfaceProvided("RobotIO");
    if (provIO) {
        provIO->AddCommandVoid(&mtsSimHWECM::EnablePower, this, "EnablePower");
        provIO->AddCommandVoid(&mtsSimHWECM::DisablePower, this, "DisablePower");

        StateTable.AddData(mActuatorPowerStatus, "ActuatorPowerStatus");
        provIO->AddCommandReadState(StateTable, mActuatorPowerStatus,
                                    "GetActuatorAmpStatus"); // vector[bool]

        StateTable.AddData(mBrakePowerStatus, "BrakePowerStatus");
        provIO->AddCommandReadState(StateTable, mBrakePowerStatus,
                                    "GetBrakeAmpStatus"); // vector[bool]

        provIO->AddCommandVoid(&mtsSimHWECM::BiasEncoder, this, "BiasEncoder");
        provIO->AddCommandWrite(&mtsSimHWECM::SetActuatorCurrent, this,
                                "SetActuatorCurrent", mActuatorCurrent);

        provIO->AddCommandVoid(&mtsSimHWECM::BrakeRelease, this, "BrakeRelease");
        provIO->AddCommandVoid(&mtsSimHWECM::BrakeEngage, this, "BrakeEngage");
    }

    // ManipClutch
    mtsInterfaceProvided* provManip = AddInterfaceProvided("ManipClutch");
    if (provManip) {
        provManip->AddEventWrite(mManipClutchEventTrigger, "Button", prmEventButton());
    }

    mtsInterfaceProvided* provSUJ = AddInterfaceProvided("SUJClutch");
    if (provSUJ) {
        provSUJ->AddEventWrite(mSUJClutchEventTrigger, "Button", prmEventButton());
    }
}


void mtsSimHWECM::Run()
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // What should I do here?
    // Publish Command Position If necessary
}


// ------------------------------------------
// PID
// ------------------------------------------

void mtsSimHWECM::Enable(const mtsBool & enable)
{
    mPIDEnabled = enable.Data;
}

void mtsSimHWECM::SetDesiredPositions(const prmPositionJointSet &prmPos)
{
    mDesiredPositionParam = prmPos;
    mDesiredPositionParam.GetGoal(mDesiredPosition);    

    // sanity check
    if (mDesiredPosition.size() != mNumJoints) {
        CMN_LOG_RUN_ERROR << "Desired Position Size Mismatch" << std::endl;
        return;
    }

    if (mCheckJointLimit) {
        // limit check: clip the desired position
        mDesiredPosition.ElementwiseMin(mJointUpperLimit);
        mDesiredPosition.ElementwiseMax(mJointLowerLimit);
    }

    // Publish
    mMsgJntCmd.position.resize(mNumJoints);
    std::copy(mDesiredPosition.begin(), mDesiredPosition.end(), mMsgJntCmd.position.begin());
    mPubECMJntCmd.publish(mMsgJntCmd);
}


// ------------------------------------------
// RobotIO
// ------------------------------------------
void mtsSimHWECM::EnablePower(void)
{
    CMN_LOG_RUN_DEBUG << "EnablePower Called" << std::endl;
    mIsPowerOn = true;
}

void mtsSimHWECM::DisablePower(void)
{
    CMN_LOG_RUN_DEBUG << "DisablePower Called" << std::endl;
    mIsPowerOn = false;
}

void mtsSimHWECM::BiasEncoder(void)
{
    CMN_LOG_RUN_DEBUG << "BiasEncoder Called" << std::endl;
}

void mtsSimHWECM::SetActuatorCurrent(const vctDoubleVec & currents)
{
    if (currents.size() != mNumJoints) {
        CMN_LOG_RUN_ERROR << "Wrong currents size" << std::endl;
    }
    mActuatorCurrent.ForceAssign(currents);
}

void mtsSimHWECM::BrakeRelease(void)
{
    CMN_LOG_RUN_DEBUG << "BrakeRelease Called" << std::endl;
    mIsBrakeReleased = true;
}

void mtsSimHWECM::BrakeEngage(void)
{
    CMN_LOG_RUN_DEBUG << "BrakeEngage Called" << std::endl;
    mIsBrakeReleased = false;
}


// ------------------------------------------
// ROS
// ------------------------------------------
void mtsSimHWECM::CallbackECMJointState(const sensor_msgs::JointState &msg)
{
    // sanity check
    vctDoubleVec jntPos(4, 0.0);
    if (msg.position.size() != 7) {
        CMN_LOG_RUN_ERROR << "ECM ROS Joint Size Error" << std::endl;
        mFeedbackPositionParam.SetPosition(jntPos);
        return;
    }

    // assign jnt pos
    jntPos[0] = msg.position[0];   // ecm_yaw_joint
    jntPos[1] = msg.position[1];   // ecm_pitch_joint
    jntPos[2] = msg.position[5];   // ecm_insertion_joint
    jntPos[3] = msg.position[6];   // ecm_roll_joint
    mFeedbackPositionParam.SetPosition(jntPos);
}


void mtsSimHWECM::CallbackECMManipClutch(const std_msgs::Bool &msg)
{
    static prmEventButton
            pressed = prmEventButton::PRESSED,
            released = prmEventButton::RELEASED;

    if (msg.data == true) {
        mManipClutchEventTrigger(pressed);
        ROS_INFO("ECM Manip Pressed");
    } else {
        mManipClutchEventTrigger(released);
        ROS_INFO("ECM Manip Released");
    }
}

void mtsSimHWECM::CallbackECMSUJClutch(const std_msgs::Bool &msg)
{
    static prmEventButton
            pressed = prmEventButton::PRESSED,
            released = prmEventButton::RELEASED;

    if (msg.data == true) {
        mSUJClutchEventTrigger(pressed);
        ROS_INFO("ECM SUJ Pressed");
    } else {
        mSUJClutchEventTrigger(released);
        ROS_INFO("ECM SUJ Released");
    }
}




