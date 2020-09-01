#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

class KeyboardTeleop:
    def __init__(self):
        rospy.init_node('keyboard_teleop')
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.msg = Twist()
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv.imshow('image', cv_image)
        except CvBridgeError as e:
            print(e)

        k = cv.waitKey(1)

        if k == ord('w'):
            self.msg.linear.x = 0.2
            self.msg.angular.x = 0
        elif k == ord('s'):
            self.msg.linear.x = -0.2
            self.msg.angular.x = 0
        elif k == ord('a'):
            self.msg.linear.x = 0.2
            self.msg.angular.z = 0.3
        elif k == ord('d'):
            self.msg.linear.x = 0.2
            self.msg.angular.z = -0.3
        elif k == ord('q'):
            rospy.signal_shutdown("shutdown")
            cv.destroyAllWindows()
        else:
            self.msg.linear.x = 0
            self.msg.angular.z = 0

        self.pub.publish(self.msg)
        self.rate.sleep()

if __name__ == '__main__':
    kt = KeyboardTeleop()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
