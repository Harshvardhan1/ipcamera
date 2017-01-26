#!/usr/bin/env python
import cv2
import urllib
import numpy as np
from sensor_msgs.msg import Image
import roslib
import sys
import rospy
#from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import argparse
import imutils

buffersize=64
pts = deque(maxlen=buffersize)
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
        else:
            rospy.loginfo("Image received")
            greenLower = (29, 86, 6)
            greenUpper = (64, 255, 255)
            cv_image = imutils.resize(cv_image, width=600)
        	# blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        	# construct a mask for the color "green", then perform
        	# a series of dilations and erosions to remove any small
        	# blobs left in the mask
            mask1 = cv2.inRange(hsv, greenLower, greenUpper)
        	## erosion followed by dilation is called opening
        	# erosion removes noise from the image and shrinks it, then dilted to recover clear image
            kernel = np.ones((5,5),np.uint8)
            mask2 = cv2.erode(mask1, None, iterations=2)
            mask = cv2.dilate(mask2, None, iterations=2)
        	#cv2.imshow("filtered",mask1)
        	#cv2.imshow("eroded",mask2)
        	#cv2.imshow("dilated",mask)
        	# find contours in the mask and initialize the current
        	# (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        	   cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None
        	# only proceed if at least one contour was found
            if len(cnts) > 0:
        		# find the largest contour in the mask, then use
        		# it to compute the minimum enclosing circle and
        		# centroid
        		c = max(cnts, key=cv2.contourArea)
        		((x, y), radius) = cv2.minEnclosingCircle(c)
        		M = cv2.moments(c)
        		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        		# only proceed if the radius meets a minimum size
        		if radius > 10:
        			# draw the circle and centroid on the cv_image,
        			# then update the list of tracked points
        			cv2.circle(cv_image, (int(x), int(y)), int(radius),
        				(0, 255, 255), 2)
        			cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

        	# update the points queue
            pts.appendleft(center)
        	# loop over the set of tracked points
            for i in xrange(1, len(pts)):
        		# if either of the tracked points are None, ignore
        		# them
        		if pts[i - 1] is None or pts[i] is None:
        			continue
        		# otherwise, compute the thickness of the line and
        		# draw the connecting lines
        		thickness = int(np.sqrt(buffersize / float(i + 1)) * 2.5)
        		cv2.line(cv_image, pts[i - 1], pts[i], (0, 0, 255), thickness)

        	# show the cv_image to our screen
            cv2.imshow("Frame", cv_image)
            key = cv2.waitKey(1) & 0xFF
        	# if the 'q' key is pressed, stop the loop
#
if __name__ == '__main__':
  ip_camera_listener = ipcamera()
  rospy.init_node('ip camera Listener', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
 # cv2.DestroyAllWindows()
