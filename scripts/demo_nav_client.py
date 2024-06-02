#!/usr/bin/env python3
# coding=utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == "__main__":
    # 生成一个导航请求客户端
    ac = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    # 等待服务器端启动
    ac.wait_for_server()

    # 构建目标航点消息
    goal = MoveBaseGoal()
    # 目标航点的参考坐标系
    goal.target_pose.header.frame_id="map"
    # 目标航点在参考坐标系里的三维数值
    goal.target_pose.pose.position.x = -3.0
    goal.target_pose.pose.position.y = 2.0
    goal.target_pose.pose.position.z = 0.0
    # 目标航点在参考坐标系里的朝向信息
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    # 发送目标航点去执行
    ac.send_goal(goal)
    rospy.loginfo("开始导航…")

    # 等待 move_base 完成导航
    ac.wait_for_result()
    
    # 获取导航结果
    if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("导航成功！")
    else:
        rospy.loginfo("导航失败…")

