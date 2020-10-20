#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time 
import math
rospy.init_node("detect_cube_node")
bridge = CvBridge()

cv_image = 0

field_min = np.array([0,0,235], dtype=np.uint8)
field_max = np.array([255,255,255], dtype=np.uint8)

red_min = np.array([0, 123, 162], dtype=np.uint8)
red_max = np.array([0, 255, 255], dtype=np.uint8)

green_min = np.array([46, 238, 0], dtype=np.uint8)
green_max = np.array([69, 255, 255], dtype=np.uint8)

blue_min = np.array([80, 80, 0], dtype=np.uint8)
blue_max = np.array([255, 255, 255], dtype=np.uint8)

yellow_min = np.array([27, 127, 40], dtype=np.uint8)
yellow_max = np.array([42, 255, 255], dtype=np.uint8)


_x_ = 0
_y_ = 1
_z_ = 2
_phi_ = 3



color_name = ["RED", "GREEN", "BLUE", "YELLOW"]
color_min  = [red_min, green_min, blue_min, yellow_min]
color_max  = [red_max, green_max, blue_max, yellow_max]

rospy.set_param("/defined_cube_colors", color_name)
if rospy.has_param("/find_cubes"):
    rospy.delete_param("/find_cubes")


def find_color(pic,color_min_range_param, color_max_range_param):
    frame = pic
    x = 0
    y = 0
    w = 0
    h = 0
    blur = cv2.blur(frame, (7,7), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv, color_min_range_param, color_max_range_param)
    if int(cv2.__version__[0]) == 3:
        _, conts, hier = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:        
        conts, hier = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    
    if conts:
        conts = sorted(conts, key=cv2.contourArea, reverse=True)
        (x, y, w, h)= cv2.boundingRect(conts[0])
        #cv2.rectangle(frame, (x,y), (x+w, y+h), (255, 0, 255), 2)
    return  frame, x, y, w, h

def find_cube(pic,color,color_min_range_param, color_max_range_param):
    frame = pic.copy()
    x = 0
    y = 0
    w = 0
    h = 0
    blur = cv2.blur(frame, (7,7), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv, color_min_range_param, color_max_range_param)
    if int(cv2.__version__[0]) == 3:
        _, conts, hier = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:        
        conts, hier = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    count = 0
    for c in conts:
        if conts:
            conts = sorted(conts, key=cv2.contourArea, reverse=True)
            # (x, y, w, h)= cv2.boundingRect(c)
            # cv2.rectangle(frame, (x,y), (x+w, y+h), (255, 0, 255), 2)
            

            rect = cv2.minAreaRect(c) # пытаемся вписать прямоугольник
            box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
            box = np.int0(box)

            centre = (rect[0][0], rect[0][1])
            area = int(rect[1][0]*rect[1][1])
            edge1 = np.int0((box[1][0] - box[0][0],box[1][1] - box[0][1]))
            edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))
            
            usedEdge = edge1
            if cv2.norm(edge2) > cv2.norm(edge1):
                usedEdge = edge2
            reference = (1,0) 

            angle = -math.acos((reference[0]*usedEdge[0] + reference[1]*usedEdge[1]) / (cv2.norm(reference) * cv2.norm(usedEdge)))


            cors =  [(pic.shape[1] / 2 - int(centre[0])) * x_factor , -(pic.shape[0] / 2 - int(centre[1])) * y_factor , 20, angle]
            rospy.set_param("/find_cubes/" + color + "/" + str(count) + "/pos", cors)
            
            # print("[INFO]: FIND CUBE", "SET PARAM:","/find_cubes/" + color + "/" + str(count) + "/pos", cors)
            rospy.loginfo("FIND CUBE SET PARAM: /find_cubes/" + color + "/" + str(count) + "/pos " + str(cors) )
            count = count + 1

def img_msg_cb(image_message):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logwarn(e)
    cv2.waitKey(1)
    #cv2.imshow("cv_image", cv_image)
    
    

def detect_cubes(img):
    global x_factor
    global y_factor
    field_frm, fx,fy,fw,fh = find_color(img, field_min, field_max)

    field_frm = field_frm[fy:fy+fh, fx:fx+fw]
    field_frm = cv2.resize(field_frm, (640,480))

    x_factor = 960 / field_frm.shape[1]
    y_factor = 800 / field_frm.shape[0]

    for i in range(len(color_name)):
        find_cube(field_frm, color_name[i], color_min[i], color_max[i])
    

    cv2.imshow("field_frm", field_frm)
    
    #cv2.imshow("green_frm", green_frm)
    


rospy.Subscriber("/poligon/camera1/image_raw", Image, img_msg_cb)

if __name__ == "__main__":
    time.sleep(1)
    detect_cubes(cv_image)
    