import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time 
import math
rospy.init_node("HSV_M_node")
bridge = CvBridge()
import cv2
import numpy as np

if __name__ == "__main__":
    def nothing(*arg):
        pass

white = [255,255,255]
black = [0,0,0]

field_min = np.array([0,0,235], dtype=np.uint8)
field_max = np.array([255,255,255], dtype=np.uint8)

# cv2.namedWindow("settings")
# cv2.createTrackbar("p1", "settings", 0, 255, nothing)
# cv2.createTrackbar("p2", "settings", 0, 255, nothing)

def find_color(pic,color_min_range_param, color_max_range_param):
    frame = pic
    x = 0
    y = 0
    w = 0
    h = 0
    blur = cv2.blur(frame, (7,7), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv, color_min_range_param, color_max_range_param)
    _, conts, hier = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if conts:
        conts = sorted(conts, key=cv2.contourArea, reverse=True)
        (x, y, w, h)= cv2.boundingRect(conts[0])
        #cv2.rectangle(frame, (x,y), (x+w, y+h), (255, 0, 255), 2)
    return  frame, x, y, w, h

def img_msg_cb(image_message):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        field_frm, fx,fy,fw,fh = find_color(cv_image, field_min, field_max)

        field_frm = field_frm[fy:fy+fh, fx:fx+fw]
        field_frm =cv2.resize(field_frm, (640,480))
        img = field_frm.copy()

        height, width, channels = img.shape
        
        for x in range(0,width):
            for y in range(0,height):
                if img[x,y,0] == 255 and img[x,y,1] == 255 and img[x,y,2] == 255:            
                    img[x,y,0] = 0
                    img[x,y,1] = 0
                    img[x,y,2] = 0

                elif img[x,y,0] == 0 and img[x,y,1] == 0 and img[x,y,2] == 0:
                    img[x,y,0] = 255
                    img[x,y,1] = 255
                    img[x,y,2] = 255

        cv2.
    except CvBridgeError as e:
        print(e)
    cv2.waitKey(0)

rospy.Subscriber("/poligon/camera1/image_raw", Image, img_msg_cb)
time.sleep(2)
while not rospy.is_shutdown():
    
    rospy.spin()
