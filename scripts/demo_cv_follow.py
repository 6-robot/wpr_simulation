#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

hue_min = 10
hue_max = 40
satu_min = 90
satu_max = 255
val_min = 1
val_max = 255

vel_cmd = Twist()
vel_pub = None

def Cam_RGB_Callback(msg):
    global hue_min, hue_max, satu_min, satu_max, val_min, val_max
    global vel_cmd, vel_pub
    
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("格式转换错误: %s", e)
        return

    # 将RGB图片转换成HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 在HSV空间做直方图均衡化
    h, s, v = cv2.split(hsv_image)
    v = cv2.equalizeHist(v)
    hsv_image = cv2.merge([h, s, v])

    # 使用Hue,Saturation和Value的阈值范围对图像进行二值化
    th_image = cv2.inRange(hsv_image, (hue_min, satu_min, val_min), (hue_max, satu_max, val_max))

    # 开操作 (去除一些噪点)
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    th_image = cv2.morphologyEx(th_image, cv2.MORPH_OPEN, element)

    # 闭操作 (连接一些连通域)
    th_image = cv2.morphologyEx(th_image, cv2.MORPH_CLOSE, element)

    # 遍历二值化后的图像数据
    target_x, target_y, pix_count = 0, 0, 0
    image_height, image_width = th_image.shape[:2]
    
    for y in range(image_height):
        for x in range(image_width):
            if th_image[y, x] == 255:
                target_x += x
                target_y += y
                pix_count += 1

    print(f"横向宽度= {image_width}   纵向高度= {image_height}")

    if pix_count > 0:
        target_x //= pix_count
        target_y //= pix_count
        print(f"颜色质心坐标( {target_x} , {target_y} )  点数 = {pix_count}")
        # 画坐标
        cv2.line(cv_image, (target_x-10, target_y), (target_x+10, target_y), (255, 0, 0), 3)
        cv2.line(cv_image, (target_x, target_y-10), (target_x, target_y+10), (255, 0, 0), 3)
        
        # 计算机器人运动速度
        vel_forward = (image_height/2 - target_y) * 0.001
        vel_turn = (image_width/2 - target_x) * 0.0005
        vel_cmd.linear.x = vel_forward
        vel_cmd.angular.z = vel_turn
    else:
        print("目标颜色消失...")
        vel_cmd.linear.x = 0
        vel_cmd.angular.z = 0

    # 显示处理结果
    cv2.imshow("RGB", cv_image)
    cv2.imshow("Result", th_image)
    cv2.waitKey(1)

    vel_pub.publish(vel_cmd)
    print(f"机器人运动速度( linear.x= {vel_cmd.linear.x:.2f} , angular.z= {vel_cmd.angular.z:.2f} )")

def nothing(x):
    pass

if __name__ == "__main__":
    rospy.init_node("demo_cv_follow", anonymous=True)
    
    rgb_sub = rospy.Subscriber("kinect2/qhd/image_color_rect", Image, Cam_RGB_Callback, queue_size=1)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=30)

    cv2.namedWindow("Threshold")
    cv2.createTrackbar("hue_min", "Threshold", hue_min, 179, nothing)
    cv2.createTrackbar("hue_max", "Threshold", hue_max, 179, nothing)
    cv2.createTrackbar("satu_min", "Threshold", satu_min, 255, nothing)
    cv2.createTrackbar("satu_max", "Threshold", satu_max, 255, nothing)
    cv2.createTrackbar("val_min", "Threshold", val_min, 255, nothing)
    cv2.createTrackbar("val_max", "Threshold", val_max, 255, nothing)

    cv2.namedWindow("RGB")
    cv2.namedWindow("Result")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        hue_min = cv2.getTrackbarPos("hue_min", "Threshold")
        hue_max = cv2.getTrackbarPos("hue_max", "Threshold")
        satu_min = cv2.getTrackbarPos("satu_min", "Threshold")
        satu_max = cv2.getTrackbarPos("satu_max", "Threshold")
        val_min = cv2.getTrackbarPos("val_min", "Threshold")
        val_max = cv2.getTrackbarPos("val_max", "Threshold")
        
        rate.sleep()

    cv2.destroyAllWindows()