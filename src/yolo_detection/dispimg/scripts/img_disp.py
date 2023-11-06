#!/usr/bin/python3
# coding:utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def callback(data):
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("frame", cv_img)
    cv2.waitKey(1)


if __name__ == '__main__':
    import sys

    print(sys.version)  # 查看python版本

    rospy.init_node('img_process_node', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber('/yolov7/yolov7/visualization', Image, callback)
    rospy.spin()
