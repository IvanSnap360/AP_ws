#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node("RT_image_node")
bridge = CvBridge()



def img_msg_cb(image_message):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
        cv2.imshow("cv_image", cv_image)
    except CvBridgeError as e:
        print(e)
    cv2.waitKey(1)


    


rospy.Subscriber("/poligon/camera1/image_raw", Image, img_msg_cb)

while not rospy.is_shutdown():
    rospy.spin()