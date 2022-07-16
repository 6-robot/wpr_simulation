#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

# IMU回调函数
def imu_callback(msg):
    quaternion = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ]
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    rospy.loginfo("机器人朝向 = %f 度",yaw*180/math.pi)

# 主函数
if __name__ == "__main__":
    rospy.init_node("imu_data")
    # 订阅IMU的数据话题
    imu_sub = rospy.Subscriber("/imu/data",Imu,imu_callback,queue_size=10)
    rospy.spin()