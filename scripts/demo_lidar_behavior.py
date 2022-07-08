#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

count = 0

# 激光雷达回调函数
def lidar_callback(msg):
    global vel_pub
    global count
    vel_msg = Twist()
    middle = len(msg.ranges)//2
    dist = msg.ranges[middle]
    rospy.logwarn("正前方测距数值 = %f 米",dist)
    if count > 0:
        count = count -1
        rospy.loginfo("持续转向 count = %d",count)
        return
    if dist > 1.5:
        vel_msg.linear.x = 0.05
    else:
        vel_msg.angular.z = 0.3
        count = 50
    vel_pub.publish(vel_msg)

# 主函数
if __name__ == "__main__":
    rospy.init_node("lidar_behavior")
    # 发布机器人运动控制话题
    vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    # 订阅激光雷达的数据话题
    lidar_sub = rospy.Subscriber("scan",LaserScan,lidar_callback,queue_size=10)
    rospy.spin()