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
        rospy.init_node('line_follower')
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.msg = Twist()
        self.bridge = CvBridge()
        self.param = {'KP': 0.005, 'KI': 0, 'KD': 0, 'SP': 0.2}
        self.intg = self.lastError = 0
        srv = Server(LineParamConfig, self.reconfig)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        error = self.process(cv_image)
        if error != None:
            balance = self.pid(error)
            self.msg.linear.x = self.param['SP']
            self.msg.angular.z = balance
            self.pub.publish(self.msg)
            self.rate.sleep()

    def reconfig(self, config, level):
        self.param = config
        return config

    def pid(self, error):
        self.intg += error
        diff = error - self.lastError
        self.lastError = error
        balance = self.param['KP'] * error + self.param['KI'] * self.intg + self.param['KD'] * diff
        return balance

    def process(self, src):
        height, width, _ = src.shape
        tl = (int(width/2*(1 - self.param['ROI_W'])), int(height * self.param['ROI_Y']))
        br = (int(width/2*(1 + self.param['ROI_W'])), int(height * (self.param['ROI_Y'] + self.param['ROI_H'])))

        rect = cv.rectangle(src, tl, br, (255, 0, 0), 2)
        roi = src[tl[1]+2:br[1]-2, tl[0]+2:br[0]-2]
        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 50, 255, cv.THRESH_BINARY_INV)
        _, contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(roi, contours, -1, (0, 255, 0), 3)
        cv.line(src, (width/2, 0), (width/2, height), (0, 0, 255), 2)

        if len(contours) > 0:
            cnt = contours[0]
            M = cv.moments(cnt)
            if M['m00'] != 0:
            	cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            	roi_ht, roi_wt, _ = roi.shape
            	cv.line(roi, (cx, cy), (roi_wt/2, cy), (255, 0, 255), 2)
            	error = roi_wt/2 - cx
            	cv.imshow("src", src)
            	cv.waitKey(1)
            	return error
        else:
            cv.imshow("src", src)
            cv.waitKey(1)

if __name__ == '__main__':
    LineFollower = LineFollower()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
