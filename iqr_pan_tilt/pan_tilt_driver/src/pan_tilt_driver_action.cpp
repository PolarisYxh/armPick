/**********************************************************************************
** MIT License
** 
** Copyright (c) 2019 I-Quotient-Robotics https://github.com/I-Quotient-Robotics
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** 
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
** 
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*********************************************************************************/
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include "pan_tilt_msgs/PanTiltCmdAction.h"
#include "pan_tilt_driver.h"

#include <actionlib/server/simple_action_server.h>

#include <string>

#include <thread>

// using namespace std;

typedef actionlib::SimpleActionServer<pan_tilt_msgs::PanTiltCmdAction> Server;

class PanTiltDriverAction
{
public:
  PanTiltDriverAction();
  ~PanTiltDriverAction();
  void JointStatePublish();
  void ExecuteCb(const pan_tilt_msgs::PanTiltCmdGoalConstPtr& goal);
  void preemptCB();
  void ExcuteThread();
  ros::NodeHandle _nh;
protected:
  // void callBack(const pan_tilt_msgs::PanTiltCmd &msg);


private:
  Server server;
  IQR::PanTiltDriver *_pt;
  std::string _portName;
  std::string _yawJointName;
  std::string _pitchJointName;
  sensor_msgs::JointState _js;
  ros::Subscriber _cmdSub;
  ros::Publisher _jointPub;
};

PanTiltDriverAction::PanTiltDriverAction() : 
  server(_nh, "pan_tilt_cmd", boost::bind(&PanTiltDriverAction::ExecuteCb, this, _1), false)
{
  // ROS_INFO("init!!!!!!!!!!!!!");
  //init params
  // _nh.param<std::string>("port_name", _portName, "/dev/ttyACM0");
  _nh.param<std::string>("port_name", _portName, "/dev/caster_head");
  _nh.param<std::string>("yaw_joint_name", _yawJointName, "iqr_pan_tilt_yaw_joint");
  _nh.param<std::string>("pitch_joint_name", _pitchJointName, "iqr_pan_tilt_pitch_joint");

  _js.name.resize(2);
  _js.position.resize(2);
  _js.velocity.resize(2);
  _js.effort.resize(2);
  _js.name[0] = _yawJointName;
  _js.name[1] = _pitchJointName;

  server.registerPreemptCallback(boost::bind(&PanTiltDriverAction::preemptCB, this));
  
  //set subscriber
  _jointPub = _nh.advertise<sensor_msgs::JointState>("/joint_states", 50);
  //set publisher
  // _cmdSub = _nh.subscribe("pan_tilt_cmd", 50, &PanTiltDriverAction::callBack, this);

  std::cout << "griapper port name:" << _portName << std::endl;
  _pt = new IQR::PanTiltDriver(_portName);
  server.start();
  sleep(2);
}

void PanTiltDriverAction::ExcuteThread() {

}

void PanTiltDriverAction::ExecuteCb(const pan_tilt_msgs::PanTiltCmdGoalConstPtr& goal)
{
  ROS_INFO("````````````````````````````````````````");
  // if (task_active_)
  // {
  //   server.setRejected();
  //   return;
  // }
  _pt->setPose(goal->yaw, goal->pitch, goal->speed);
  ros::WallDuration(10.0).sleep();
  if (server.isActive())
  {
    server.setSucceeded();
  }
  // task_active_ = true;
  ROS_INFO("call pick task server");

  // std::thread thread(&PickTaskAction::ExcuteThread, this, gh);
  // thread.detach();
  // task_active_ = false;
}

void PanTiltDriverAction::preemptCB()
{
  ROS_INFO("cancel");

  // _pt->stop();
  _pt->releasePanTilt();
  // _pt->holdPanTilt();
    // set the action state to preempted
  server.setPreempted();
  // task_active_ = false;
}



PanTiltDriverAction::~PanTiltDriverAction()
{
  delete _pt;
}

void PanTiltDriverAction::JointStatePublish()
{
  float yawDeg = 0.0;
  float pitchDeg = 0.0;
  _pt->getPose(yawDeg, pitchDeg);

  double yawRad = 0.01745329 * yawDeg;
  double pitchRad = 0.01745329 * pitchDeg;

  _js.header.stamp = ros::Time::now();
  _js.position[0] = yawRad;
  _js.position[1] = pitchRad;
  _jointPub.publish(_js);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pan_tilt_driver");

  PanTiltDriverAction nd;

  // ros::Rate r(10);

  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   nd.JointStatePublish();
  //   r.sleep();
  // }

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
