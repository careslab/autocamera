/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *  ** test/

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
using namespace std;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr& msgs)
{
//  system("touch /home/shahab/whateva.whatevs");
}
// %EndTag(CALLBACK)%

void jointPositionCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

//sensor_msgs::JointState::position.op
//  msg->position.be
//  for ( msg->position.begin(); i != msg->position.end(); ++i){
//	  cout << "hello" << endl;
//	  cout << ' '<< msg->position.back() << ' ' << msg->position.front() ;
//  }
//  cout << msg->position.data();
  cout << string(50, '\n');
  cout << "hello" << endl;
  int size = msg->position.size();
  for (int i = 0; i < size; i++)
	  cout << msg->position.at(i) << ' ';
  cout << endl;

//  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

//void SetDesiredPositions(const prmPositionJointSet &prmPos, ros::Publisher mPubECMJntCmd)
//{
//	prmPositionJointSet mDesiredPositionParam;
//	vctDoubleVec mDesiredPosition;
//	sensor_msgs::JointState mMsgJntCmd;
//	int mNumJoints = 4;
//	mMsgJntCmd.name.clear();
//	mMsgJntCmd.name.push_back("blahecm_yaw_joint");
//	mMsgJntCmd.name.push_back("blahecm_pitch_joint");
//	mMsgJntCmd.name.push_back("blahecm_insertion_joint");
//	mMsgJntCmd.name.push_back("blahecm_roll_joint");
//
//	mDesiredPosition.SetSize(mNumJoints);
//	mDesiredPosition.SetAll(2);
//
//    mDesiredPositionParam = prmPos;
//    mDesiredPositionParam.GetGoal(mDesiredPosition);
//
//    // sanity check
//    if (mDesiredPosition.size() != mNumJoints) {
//        cout << "Desired Position Size Mismatch" << endl;
//        return;
//    }
//
//
//    // Publish
//    mMsgJntCmd.position.resize(mNumJoints);
//    std::copy(mDesiredPosition.begin(), mDesiredPosition.end(), mMsgJntCmd.position.begin());
//    mPubECMJntCmd.publish(mMsgJntCmd);
//}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "autocamera");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  cout << "before" << endl;

  //%Tag(SUBSCRIBER)%
//  ros::Subscriber sub = n.subscribe("/dvrk_ecm/joint_states", 1, jointPositionCallback);
  //%EndTag(SUBSCRIBER)%

  sensor_msgs::JointState jnt;
  jnt.name.clear();
  jnt.name.push_back("shahab_ecm_yaw_joint");
  jnt.name.push_back("shahab_ecm_pitch_joint");
  jnt.name.push_back("shahab_ecm_insertion_joint");
  jnt.name.push_back("shahab_ecm_roll_joint");

  for(int i=0; i < 4; i++)
	  jnt.position.push_back(i);

  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/dvrk_ecm/joint_states_robot",1);
  cout << "after" << endl;



  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
//  pub.publish(jnt);
  ros::spin();
  while (!ros::isShuttingDown()){
	  cout<<"sup" << endl;
	  jnt.header.stamp = ros::Time::now();
	  jnt.position[0] = 1;
	  jnt.position[1] = 2;
	  jnt.position[2] = 1;
	  jnt.position[3] = 0;
	  pub.publish(jnt);
  }
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
