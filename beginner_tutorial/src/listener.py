#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

class Listener:
    def __init__(self):
        rospy.init_node('listener')
        self.sub = rospy.Subscriber('/topic', Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.perform(cv_image)
        except CvBridgeError as e:
            print(e)

    def perform(self, image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        cv.imshow('gray', gray)
        cv.waitKey(10)

if __name__ == '__main__':
    listener = Listener()
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
