/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class TeleopJoy
{
public:
  TeleopJoy();
  bool bStart;
  float lx;
  float ly;
  float ry;
  ros::NodeHandle n;
  ros::Subscriber sub;

  ros::Time current_time;
  ros::Time last_time;
  ros::Publisher velcmd_pub;
  void SendVelcmd();
private:
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
};

TeleopJoy::TeleopJoy()
{
  lx = 0;
  ly = 0;
  ry = 0;
  bStart = false;
  velcmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  sub = n.subscribe<sensor_msgs::Joy>("joy",10,&TeleopJoy::callBack,this);
  current_time = ros::Time::now();
  last_time = ros::Time::now();
 
  ROS_INFO("TeleopJoy");
}

static float kl = 0.2;
static float kt = 0.5;
void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{

  //ROS_INFO("Joy: [%.2f , %.2f]", lx , ry);
  lx = joy->axes[1];  //forward & backward
  ly = joy->axes[0];  //shift
  ry = joy->axes[3];

  bStart = true;
}

void TeleopJoy::SendVelcmd()
{
  if(bStart == false)
    return;
  geometry_msgs::Twist vel_cmd;
  vel_cmd.linear.x = (float)lx*kl;
  vel_cmd.linear.y = (float)ly*kl;
  vel_cmd.linear.z = 0;
  vel_cmd.angular.x = 0;
  vel_cmd.angular.y = 0;
  vel_cmd.angular.z = (float)ry*kt;
  velcmd_pub.publish(vel_cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_js_node");

  TeleopJoy cTeleopJoy;

  ros::Rate r(30);
  while(ros::ok())
  {
    cTeleopJoy.SendVelcmd();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}