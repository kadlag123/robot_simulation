#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from line_follower.cfg import LineParamConfig
from dynamic_reconfigure.server import Server

class LineFollower:
    def __init__(self):
        cv.namedWindow('image', cv.WINDOW_NORMAL)
        self.param = dict()
        srv = Server(LineParamConfig, self.reconfig)

        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        self.msg = Twist()
        self.bridge = CvBridge()

        self.intg = self.lastError = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        error = self.process(cv_image)
        balance = self.pid(error)
        self.msg.linear.x = self.param['SP']
        self.msg.angular.z = balance
        self.pub.publish(self.msg)
        self.rate.sleep()

    def process(self, image):
        width, height, channel = image.shape

        roi_w = int(self.param['ROI_W'] * width)
        roi_h = int(self.param['ROI_H'] * height)
        roi_y = int(self.param['ROI_Y'] * height)

        tl = ((width - roi_w)//2, roi_y)
        br = ((width + roi_w)//2, roi_y + roi_h)

        roi = image[tl[1]:br[1], tl[0]:br[0]]

        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 50, 255, cv.THRESH_BINARY_INV)
        _, contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        cv.rectangle(image, tl, br, (0,0,255), 2)
        cv.drawContours(roi, contours, -1, (0, 255, 255), 3)
        cv.line(image, (width//2, tl[1]), (width//2, br[1]), (255, 0, 0), 2)

        for cnt in contours:
            mu = cv.moments(cnt)
            mc = (int(mu['m10']/(mu['m00'] + 1e-5)), int(mu['m01']/(mu['m00'] + 1e-5)))

            cv.circle(roi, mc, 5, (0,0,255), -1)
            cv.line(roi, mc, (roi_w//2, mc[1]), (255,0,255), 3)
            cv.imshow('image', image)
            cv.waitKey(1)
            print(width//2, mc)
            error = width//2 - mc[0]
            return error

        cv.imshow('image', image)
        cv.waitKey(1)
        return 0

    def reconfig(self, config, level):
        self.param = config
        return config

    def pid(self, error):
        print(error)
        prop = error
        self.intg -= error
        diff = error - self.lastError
        self.lastError = error
        balance = self.param['KP'] * prop + self.param['KI'] * self.intg + self.param['KD'] * diff
        return balance

if __name__ == '__main__':
    lineFollower = LineFollower()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
