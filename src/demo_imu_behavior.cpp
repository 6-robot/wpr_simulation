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

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"

// 速度消息发布对象（全局变量）
ros::Publisher vel_pub;

// IMU 回调函数
void IMUCallback(const sensor_msgs::Imu msg)
{
    // 检测消息包中四元数数据是否存在
    if(msg.orientation_covariance[0] < 0)
        return;
    // 四元数转成欧拉角
    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // 弧度换算成角度
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI;
    ROS_INFO("滚转= %.0f 俯仰= %.0f 朝向= %.0f", roll, pitch, yaw);
    // 速度消息包
    geometry_msgs::Twist vel_cmd;
    // 目标朝向角
    double target_yaw = 90;
    // 计算速度
    double diff_angle = target_yaw - yaw;
    vel_cmd.angular.z = diff_angle * 0.01;
    vel_cmd.linear.x = 0.1;
    vel_pub.publish(vel_cmd);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc,argv, "demo_imu_behavior"); 

    ros::NodeHandle n;
    // 订阅 IMU 的数据话题
    ros::Subscriber sub = n.subscribe("imu/data", 100, IMUCallback);
    // 发布速度控制话题
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::spin();

    return 0;
}
