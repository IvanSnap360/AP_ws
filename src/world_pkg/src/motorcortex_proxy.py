#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2

import time
import motorcortex
import mcx_tracking_cam_pb2 as tracking_cam_msg

import threading
import sys

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
# pub_bar = rospy.Publisher('bar', Bool, queue_size=1)
cvBridge = CvBridge()
req = 0


def onBlob(val):
	try:
		blobs = tracking_cam_msg.Blobs()
		if blobs.ParseFromString(val[0].value):
			print(blobs.value[0].cx, blobs.value[0].cy)
			print(type(blobs.value[0].cx), type(blobs.value[0].cy))
			blob.x = float(blobs.value[0].cx)
			blob.y = float(blobs.value[0].cy)
			blob.z = 0.0
			pub_blob.publish(blob)
	except Exception as e:
	    print(e)


def imageParsing(data):
	cv_image_original = cvBridge.imgmsg_to_cv2(data, "bgr8")
	retval, buffer = cv2.imencode('.jpg', cv_image_original)
	try:
        	handle = req.setParameter("root/Processing/image", buffer.tobytes())
        	res = handle.get()
        	print("res: {}".format(req1))
	except Exception:
		print("Unexpected error imageParsing:", sys.exc_info()[0])


if __name__ == '__main__':

	# Creating empty object for parameter tree
	parameter_tree = motorcortex.ParameterTree()

	# Loading protobuf types and hashes
	motorcortex_types = motorcortex.MessageTypes()


	# Open request connection
	req_, sub_ = motorcortex.connect("wss://localhost:5568:5567", motorcortex.MessageTypes(), motorcortex.ParameterTree(),
	                               certificate="/home/nuc/Camera/mcx-tracking-cam/config/motorcortex.crt", timeout_ms=1000,
	                               login="root", password="vectioneer")
	req = req_

	rospy.init_node('motorcortex_proxy')
	sub_image = rospy.Subscriber('/poligon/camera1/image_raw', Image, imageParsing, queue_size=1)
	pub_blob = rospy.Publisher('blob', Vector3)
	blob = Vector3()
	subscription2 = sub_.subscribe(["root/Processing/BlobDetector/blobBuffer"], "blob", 1)
	subscription2.get()
	subscription2.notify(onBlob)
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()
