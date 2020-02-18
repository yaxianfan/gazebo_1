#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np 
from math import *
from geometry_msgs.msg import Pose

ball_color = 'green'

color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              }

 
class Image_converter:
    def __init__(self):

	self.bridge = CvBridge()

	self.image_pub = rospy.Publisher('table_detect_test',Image,queue_size = 10)
	
	self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.callback)
    
    	# Allow up to one second to connection
        rospy.sleep(1)

    def callback(self,data):
		
		# Convert image to OpenCV format，
		try:
		    cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
		    print e

		detect_image = self.detect_table(cv_image)

		try:
	            self.image_pub.publish(self.bridge.cv2_to_imgmsg(detect_image, "bgr8"))
	        except CvBridgeError as e:
	            print e


    def detect_table(self,image):

	
		g_image = cv2.GaussianBlur(image, (5, 5), 0)		
		hsv = cv2.cvtColor(g_image, cv2.COLOR_BGR2HSV)          
                erode_hsv = cv2.erode(hsv, None, iterations=2)                
                inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
                cnts = cv2.findContours(inRange_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

                c = max(cnts, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                cv2.drawContours(image, [np.int0(box)], -1, (0, 255, 255), 2)

		return image

    
	

if __name__ == "__main__":
    rospy.init_node("vision_manager")
    rospy.loginfo("start")
    Image_converter()
    rospy.spin()

