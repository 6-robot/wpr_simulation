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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <vector>

static ros::Publisher base_to_torso_pub;
static ros::Publisher torso_to_upperarm_pub;
static ros::Publisher upperarm_to_forearm_pub;
static ros::Publisher forearm_to_palm_pub;
static ros::Publisher palm_left_finger_pub;
static ros::Publisher left_finger_tip_pub;
static ros::Publisher palm_right_finger_pub;
static ros::Publisher right_finger_tip_pub;
static geometry_msgs::Pose2D pose_reset;
static geometry_msgs::Pose2D pose_cur;
static  tf::Transform tf_odom_reset;
static tf::TransformBroadcaster* tf_broadcast;

using namespace std;
typedef struct
{
    float fGapSize;
    int nGripperPos;
    double fPosePerMM;
}stGripperPos;
static vector<stGripperPos> arGripperPos;

float CalGripperSize(float inJointPos);
void ManiGripper(float inGripperValue)
{
    float fGripperAngle = asin((inGripperValue - 0.087)*4);
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = fGripperAngle;
    palm_left_finger_pub.publish(gripper_msg);
    right_finger_tip_pub.publish(gripper_msg);
     gripper_msg.data = -fGripperAngle;
    left_finger_tip_pub.publish(gripper_msg);
    palm_right_finger_pub.publish(gripper_msg);
}

void JointCtrlCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_WARN("JointCtrlCallback");
    std_msgs::Float64 joint_pos_msg;
    int nNumJoint = msg->position.size();
    for(int i=0;i<nNumJoint;i++)
    {
        if(msg->name[i] == "base_to_torso")
        {
            //脊柱升降
           joint_pos_msg.data = msg->position[i] ;
           base_to_torso_pub.publish(joint_pos_msg);
        }
        if(msg->name[i] == "torso_to_upperarm")
        {
            //手臂根关节
            joint_pos_msg.data = -msg->position[i];
            torso_to_upperarm_pub.publish(joint_pos_msg);
        }
        if(msg->name[i] == "upperarm_to_forearm")
        {
            //手臂肘关节
            joint_pos_msg.data = msg->position[i];
            upperarm_to_forearm_pub.publish(joint_pos_msg);
        }
        if(msg->name[i] == "forearm_to_palm")
        {
            //手腕关节
            joint_pos_msg.data = msg->position[i];
            forearm_to_palm_pub.publish(joint_pos_msg);
        }
        if(msg->name[i] == "gripper")
        {
             //手爪
            float fGripperVal = CalGripperSize(msg->position[i]);
            ManiGripper(fGripperVal);
        }
        //ROS_INFO("[wpr1_mani_cb] %d - %s = %.2f", i, msg->name[i].c_str(),msg->position[i]);
    }
}

static geometry_msgs::Pose2D pose_diff_msg;
void CtrlCallback(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("pose_diff reset");
    if( nFindIndex >= 0 )
    {
        pose_diff_msg.x = 0;
        pose_diff_msg.y = 0;
        pose_diff_msg.theta = 0;
        //ROS_WARN("[pose_diff reset]");
        pose_reset.x = pose_cur.x;
        pose_reset.y = pose_cur.y;
        pose_reset.theta = pose_cur.theta;
        tf_odom_reset.setOrigin( tf::Vector3(pose_reset.x, pose_reset.y, 0.0) );
        tf_odom_reset.setRotation( tf::createQuaternionFromRPY(0,0,pose_reset.theta) );
    }
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_cur.x = msg->pose.pose.position.x;
    pose_cur.y = msg->pose.pose.position.y;
    tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    pose_cur.theta = tf::getYaw(q);
    //ROS_WARN("[Odom]  ( %.2f  %.2f )   %.2f",pose_cur.x,pose_cur.y,pose_cur.theta);
    tf_broadcast->sendTransform(tf::StampedTransform(tf_odom_reset, ros::Time::now(), "/odom", "/pose_reset"));
}

void InitGripperPosVal()
{
    stGripperPos tmpGP;
    tmpGP.fGapSize = 0;
    tmpGP.nGripperPos = 38000;
    tmpGP.fPosePerMM = 0;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.006;
    tmpGP.nGripperPos = 35000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.020;
    tmpGP.nGripperPos = 30000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.035;
    tmpGP.nGripperPos = 25000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.049;
    tmpGP.nGripperPos = 20000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.063;
    tmpGP.nGripperPos = 15000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.076;
    tmpGP.nGripperPos = 10000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.087;
    tmpGP.nGripperPos = 5000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.099;
    tmpGP.nGripperPos = 0;
    arGripperPos.push_back(tmpGP);

    int nNumGP = arGripperPos.size();
    for(int i=1;i<nNumGP;i++)
    {
        double fDiffSize = fabs(arGripperPos[i].fGapSize - arGripperPos[i-1].fGapSize);
        int nDiffPos = fabs(arGripperPos[i].nGripperPos - arGripperPos[i-1].nGripperPos);
        arGripperPos[i].fPosePerMM = fDiffSize/nDiffPos;
    }
}

float CalGripperSize(float inJointPos)
{
    float retVal = 0;
    if(inJointPos > arGripperPos[0].nGripperPos )
    {
        retVal = 0;
    }
    else if(inJointPos <  arGripperPos[arGripperPos.size()-1].nGripperPos)
    {
        retVal = arGripperPos[arGripperPos.size()-1].fGapSize; 
    }
    else
    {
        for(int i=1;i<arGripperPos.size()-1;i++)
        {
            if(inJointPos > arGripperPos[i+1].fGapSize && inJointPos <= arGripperPos[i-1].fGapSize)
            {
                retVal = arGripperPos[i-1].fGapSize + (inJointPos-arGripperPos[i+1].fGapSize)*inJointPos-arGripperPos[i+1].fPosePerMM;
                break;
            }
        }
    }
    retVal *= 10;
    return retVal;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr1_sim");
    ros::NodeHandle n;
    tf_broadcast = new tf::TransformBroadcaster;
    tf_odom_reset.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf_odom_reset.setRotation( tf::createQuaternionFromRPY(0,0,0) );
    tf::TransformListener listener;
    tf::StampedTransform tf_diff;
    InitGripperPosVal();
    
    base_to_torso_pub = n.advertise<std_msgs::Float64>("/wpr1/base_to_torso_position_controller/command",10);
    torso_to_upperarm_pub = n.advertise<std_msgs::Float64>("/wpr1/torso_to_upperarm_position_controller/command",10);
    upperarm_to_forearm_pub = n.advertise<std_msgs::Float64>("/wpr1/upperarm_to_forearm_position_controller/command",10);
    forearm_to_palm_pub = n.advertise<std_msgs::Float64>("/wpr1/forearm_to_palm_position_controller/command",10);
    palm_left_finger_pub = n.advertise<std_msgs::Float64>("/wpr1/palm_left_finger_position_controller/command",10);
    left_finger_tip_pub = n.advertise<std_msgs::Float64>("/wpr1/left_finger_tip_position_controller/command",10);
    palm_right_finger_pub = n.advertise<std_msgs::Float64>("/wpr1/palm_right_finger_position_controller/command",10);
    right_finger_tip_pub = n.advertise<std_msgs::Float64>("/wpr1/right_finger_tip_position_controller/command",10);

    ros::Subscriber joint_ctrl_sub = n.subscribe("/wpr1/joint_ctrl",30,&JointCtrlCallback);
    ros::Subscriber ctrl_sub = n.subscribe("/wpr1/ctrl",10,&CtrlCallback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, OdomCallback);
    ros::Publisher pose_diff_pub = n.advertise<geometry_msgs::Pose2D>("/wpr1/pose_diff",1);
    pose_diff_msg.x = 0;
    pose_diff_msg.y = 0;
    pose_diff_msg.theta = 0;

    sleep(10);

    std_msgs::Float64 joint_pos_msg;
    joint_pos_msg.data = 0.25;
    base_to_torso_pub.publish(joint_pos_msg);
    //手臂根关节
    joint_pos_msg.data =  1.57;
    torso_to_upperarm_pub.publish(joint_pos_msg);
    //手臂肘关节
    joint_pos_msg.data =  -0.7;
    upperarm_to_forearm_pub.publish(joint_pos_msg);
    //手腕关节
    joint_pos_msg.data = 0;
    forearm_to_palm_pub.publish(joint_pos_msg);
    //手爪
    ManiGripper(0.05);
    ROS_WARN("wpr1_joints_init");

    ros::Rate r(20.0);

    while(n.ok())
    {
        bool res = listener.waitForTransform("/pose_reset","/base_footprint",ros::Time(0), ros::Duration(1.0));
        if(res == true)
        {
            listener.lookupTransform("/pose_reset","/base_footprint",ros::Time(0),tf_diff);

            pose_diff_msg.x = tf_diff.getOrigin().x();
            pose_diff_msg.y = tf_diff.getOrigin().y();
            tf::Quaternion pose_quat = tf_diff.getRotation ();
            float pose_yaw = tf::getYaw(pose_quat);
            pose_diff_msg.theta = pose_yaw - pose_reset.theta;
            pose_diff_pub.publish(pose_diff_msg);
            //ROS_INFO("[pose_diff_msg] x= %.2f  y=%.2f  th= %.2f", pose_diff_msg.x,pose_diff_msg.y,pose_diff_msg.theta);
        }

        ros::spinOnce();
        r.sleep();
    }
    delete tf_broadcast;
}