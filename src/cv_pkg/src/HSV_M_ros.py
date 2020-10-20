#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time 
import math
rospy.init_node("HSV_M_node")
bridge = CvBridge()
import cv2
import numpy as np



field_min = np.array([0,0,235], dtype=np.uint8)
field_max = np.array([255,255,255], dtype=np.uint8)

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

if __name__ == "__main__":
    def nothing(*arg):
        pass

cv2.namedWindow("settings") 




cv2.createTrackbar("h1", "settings", 0, 255, nothing)
cv2.createTrackbar("s1", "settings", 0, 255, nothing)
cv2.createTrackbar("v1", "settings", 0, 255, nothing)
cv2.createTrackbar("h2", "settings", 255, 255, nothing)
cv2.createTrackbar("s2", "settings", 255, 255, nothing)
cv2.createTrackbar("v2", "settings", 255, 255, nothing)


def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min



def process(frame):
    global lower_color
    global upper_color

    h1 = cv2.getTrackbarPos("h1", "settings")
    s1 = cv2.getTrackbarPos("s1", "settings")
    v1 = cv2.getTrackbarPos("v1", "settings")
    h2 = cv2.getTrackbarPos("h2", "settings")
    s2 = cv2.getTrackbarPos("s2", "settings")
    v2 = cv2.getTrackbarPos("v2", "settings")
    
    
    lower_color = np.array([h1,s1,v1], np.uint8)
    upper_color = np.array([h2, s2, v2], np.uint8)
 
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.blur(hsv, (7, 7))    
    mask = cv2.inRange(mask, lower_color, upper_color)
    res = contours(mask, frame)
    res = cv2.bitwise_and(frame, frame, mask = mask)

    #cv2.imshow("frame",frame)
    #cv2.imshow("mask",mask)
    
    cv2.imshow("settings",res)
    

def img_msg_cb(image_message):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")

        field_frm, fx,fy,fw,fh = find_color(cv_image, field_min, field_max)

        field_frm = field_frm[fy:fy+fh, fx:fx+fw]
        field_frm =cv2.resize(field_frm, (640,480))

        process(field_frm)
        
    except CvBridgeError as e:
        print(e)
    cv2.waitKey(1)
    
       
def contours(thresh,frame):
    if int(cv2.__version__[0]) == 3:
        _, conts, hier = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:        
        conts, hier = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    
    for c in conts:
        if conts:
            conts = sorted(conts, key=cv2.contourArea, reverse=True)
            rect = cv2.minAreaRect(c) # пытаемся вписать прямоугольник
            box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
            box = np.int0(box)

            centre = (rect[0][0], rect[0][1])
            #area = int(rect[1][0]*rect[1][1])
            edge1 = np.int0((box[1][0] - box[0][0],box[1][1] - box[0][1]))
            edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

            cv2.drawContours(frame,[box],0,(255,0,255),4)

            return frame
            
            



rospy.Subscriber("/poligon/camera1/image_raw", Image, img_msg_cb)
time.sleep(2)
color = str(input("Enter sought color (string): "))
while not rospy.is_shutdown():
    
    rospy.spin()

# lower_color = np.array([h1,s1,v1], np.uint8)
# upper_color = np.array([h2, s2, v2], np.uint8)
print()
print(color + "_min = np.array([" + str(lower_color[0]) + ", "+ str(lower_color[1]) + ", " + str(lower_color[2]) + "], dtype=np.uint8)" )
print(color + "_max = np.array([" + str(upper_color[0]) + ", "+ str(upper_color[1]) + ", " + str(upper_color[2]) + "], dtype=np.uint8)" )

cv2.destroyAllWindows()