#!/usr/bin/env python
import cv2
import urllib
import numpy as np
from sensor_msgs.msg import Image
import roslib
import sys
import rospy
import cv
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class ipcamera(object):
    def __init__(self):
        #cv.NamedWindow("Space Cam Listener", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera_image",Image,self.callback)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        cv2.imshow("Stream: webcam", cv_image)
        rospy.loginfo("Image received")
        cv.WaitKey(3)

if __name__ == '__main__':
  ip_camera_listener = ipcamera()
  rospy.init_node('ip camera Listener', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()
