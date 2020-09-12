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

    def process(self, image):
        # TODO: Do it youtself
        pass

    def reconfig(self, config, level):
        self.param = config
        return config

    def pid(self, error):
        prop = error
        self.intg += error
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
