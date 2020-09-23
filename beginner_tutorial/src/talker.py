#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

class Talker:
    def __init__(self):
        rospy.init_node('talker')
        self.pub = rospy.Publisher('/topic', Image, queue_size=1)
        self.rate = rospy.Rate(10)  #10Hz
        self.msg = Image()
        self.bridge = CvBridge()

        self.read()

    def read(self):
        cap = cv.VideoCapture('Videos/igv.mp4')
        while not rospy.is_shutdown():
            ret, image = cap.read()
            if ret:
                cv.imshow('image', image)
                if cv.waitKey(10) & 0xFF == ord('q'):
                    break
                self.msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
                self.publish()
        cap.release()
        cv.destroyAllWindows()

    def publish(self):
        self.pub.publish(self.msg)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        talker = Talker()
    except rospy.ROSInterruptException as e:
        print(e)
