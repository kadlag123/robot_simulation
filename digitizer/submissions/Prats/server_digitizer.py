#!/usr/bin/env python
import apriltag
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig
import numpy as np
import math
import random as rng

rng.seed(12345)

class Digitizer:
    def __init__(self):
        rospy.init_node('dynamic_tutorials')
        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #self.pub2 = rospy.Publisher('/topic', Float32, queue_size=10)
        #self.rate = rospy.Rate(10)

        self.error = Float32()
        self.msg = Twist()
        self.bridge = CvBridge()

        self.param = {'ROI_R': 44, 'ROI_W': 50, 'ROI_H': 30}
        #--------apriltag----------
        self.img_org=  cv.imread('/home/pratyush/Desktop/apriltag2.jpeg',cv.IMREAD_COLOR)
        #self.img = cv.imread('/home/pratyush/Desktop/apriltag2.jpeg',cv.IMREAD_GRAYSCALE)
        self.img=  cv.imread('/home/pratyush/Desktop/way1.jpg',cv.IMREAD_GRAYSCALE)
        self.detector = apriltag.Detector()   
        #self.intg = self.lastError = 0
        srv = Server(TutorialsConfig, self.reconfig)
        cv.imshow("apriltag",self.img)
        cv.waitKey(1)

    def callback(self, data):
        
        frame=self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        height,width,channel=frame.shape
        #print(width,height)
        w = self.param['ROI_W']
        h = self.param['ROI_H']
        r = self.param['ROI_R']        
        detector = apriltag.Detector()
        
        result = detector.detect(gray)
        src=frame
        
        for i in range(0,len(result[0].corners)):
            cv.circle(src,(int(result[0].corners[i][0]),int(result[0].corners[i][1])),8,(0,255,0),-1)
            cv.circle(src,(int(result[0].center[0]),int(result[0].center[1])),8,(0,0,255),-1)
            cv.putText(src, str(i+1), (int(result[0].corners[i][0]),int(result[0].corners[i][1])), cv.FONT_HERSHEY_SIMPLEX,0.4, (0, 0, 0), 2)
        m=(-1)*(float)(result[0].corners[3][0]-result[0].corners[0][0])/(result[0].corners[3][1]-result[0].corners[0][1])
        m1=(-1)*(float)((result[0].corners[1][1]-result[0].corners[0][1])/(result[0].corners[1][0]-result[0].corners[0][0]))
        pm1=(-1)*(1/m1)
        m2=(-1)*(float)((result[0].corners[2][1]-result[0].corners[1][1])/(result[0].corners[2][0]-result[0].corners[1][0]))  

        #print("side slope m:",m)
        #print("side slode m1:",m1)
        #print("angle is",(-1)*math.atan2((result[0].corners[1][1]-result[0].corners[0][1]),(result[0].corners[1][0]-result[0].corners[0][0]))*180/3.14)
        #print("cos is" ,math.cos(math.atan(m1)))
        #print("sin is" ,math.sin(math.atan(m1)))
        #print("pm is",pm1)
        #cv.circle(src,(int(result[0].center[0]),int(result[0].center[1])),8,(0,0,255),-1) 
        #angle=(-1)*math.atan2((result[0].corners[1][1]-result[0].corners[0][1]),(result[0].corners[1][0]-result[0].corners[0][0]))*180/3.14
        angle_r=(-1)*math.atan2((result[0].corners[1][1]-result[0].corners[0][1]),(result[0].corners[1][0]-result[0].corners[0][0]))
        #angle_r_p=(-1)/angle_r
        angle_r_2=(-1)*math.atan2((result[0].corners[2][1]-result[0].corners[1][1]),(result[0].corners[2][0]-result[0].corners[1][0]))
        #print("per angle is",angle_r_p)
        #print("m2 is ",angle_r_2)
        cx=(result[0].center[0])
        cy=(result[0].center[1])
        new_target_x=cx+(r*math.cos(angle_r))
        new_target_y=cy-(r*math.sin(angle_r))
        new_target_x_d=cx+((r-h/2)*math.cos(angle_r))
        new_target_y_d=cy-((r-h/2)*math.sin(angle_r))

        trial_1_x=new_target_x+(w/2*math.cos(angle_r_2)) 
        trial_1_y=new_target_y-(w/2*math.sin(angle_r_2))
        trial_2_x=new_target_x-(w/2*math.cos(angle_r_2))
        trial_2_y=new_target_y+(w/2*math.sin(angle_r_2))
        trial_3_x=new_target_x_d+(w/2*math.cos(angle_r_2)) 
        trial_3_y=new_target_y_d-(w/2*math.sin(angle_r_2))
        trial_4_x=new_target_x_d-(w/2*math.cos(angle_r_2))
        trial_4_y=new_target_y_d+(w/2*math.sin(angle_r_2))
        #cv.circle(src,(int(new_target_x),int(new_target_y)),8,(0,255,0),-1)
       
        cv.circle(src,(int(trial_1_x),int(trial_1_y)),8,(0,255,0),-1)
        cv.circle(src,(int(trial_2_x),int(trial_2_y)),8,(100,255,0),-1)
        cv.circle(src,(int(trial_3_x),int(trial_3_y)),8,(0,255,0),-1)
        cv.circle(src,(int(trial_4_x),int(trial_4_y)),8,(0,255,0),-1)
        pts1 = np.float32([[trial_2_x,trial_2_y],[trial_1_x,trial_1_y],[trial_3_x,trial_3_y],[trial_4_x,trial_4_y]])
        pts2 = np.float32([[0,0],[w*10,0],[w*10,(h/2)*12],[0,(h/2)*12]]) #scale it
        matrix = cv.getPerspectiveTransform(pts1,pts2)
        result = cv.warpPerspective(frame,matrix,(int(h/2)*12,int(w)*10))
        sk_result=cv.warpPerspective(self.img,matrix,(int(h/2)*12,int(w)*10))

        _, contours, _ = cv.findContours(sk_result, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        l_circle=[]
        cv.imshow("perspective",result)
        cv.imshow("perspective2",sk_result)
        cv.imshow("apriltag",src)
        cv.imshow('Contours', frame)
        cv.waitKey(1)
        def sortsecond(val):
           return val[1]
        contours_poly = [None]*len(contours)
        boundRect = [None]*len(contours)
        centers = [None]*len(contours)
        radius = [None]*len(contours)
        if len(contours)>0:
          for i, c in enumerate(contours):
             contours_poly[i] = cv.approxPolyDP(c, 3, True)
             boundRect[i] = cv.boundingRect(contours_poly[i])
             centers[i], radius[i] = cv.minEnclosingCircle(contours_poly[i])
             l_circle.append((i,radius[i]))

          l_circle.sort(reverse=True,key=sortsecond)
          rows,cols=sk_result.shape 
    
          cv.circle(sk_result,(int(centers[l_circle[0][0]][0]),int(centers[l_circle[0][0]][1])),8,(0,0,255),-1)
          cv.circle(sk_result,(int(rows/2),int(cols/2)),8,(0,255,0),-1)               

          for i in range(len(contours)):
              color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
              cv.drawContours(sk_result, contours_poly, i, color)
              cv.rectangle(sk_result, (int(boundRect[i][0]), int(boundRect[i][1])), \
                (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)
              cv.circle(sk_result, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)
          shift=centers[l_circle[0][0]][0]-rows/2
        
          print("shift is",shift)
          shift_abs=abs(centers[l_circle[0][0]][0]-rows/2)
            #to move bot in forward direction
          if shift_abs<=50:
              self.msg.linear.x=0.05   #0.15
              self.msg.angular.x=0
            #to rotate the bot
          elif shift<0 and shift_abs<=500:
              self.msg.angular.z=0.75
              self.msg.linear.x=0.025
              rospy.Duration(2)
          elif shift>0 and shift_abs<=500:
              self.msg.angular.z=-0.1
              self.msg.linear.x=0.025
              rospy.Duration(2)
          elif shift_abs>=500:   #203
              self.msg.angular.z=0
              self.msg.linear.x=-0.1  #had changed values earlier         
          else:
              self.msg.angular.z=0
              self.msg.linear.x=0 
          self.pub.publish(self.msg)
        else:
          print("no track found yet")
          self.msg.angular.z=-0.06
          self.msg.linear.x=0 
          self.pub.publish(self.msg)

    def process(self, image):
        pass


    def reconfig(self, config, level):
        self.param = config
        print(self.param)
        result = self.detector.detect(self.img)
        print(result)
        #cv.imshow("apriltag",self.img)
        #cv.waitKey(1)  
        #cv.destroyAllWindows()
        return config

    def pid(self, error):
        pass

if __name__ == '__main__':
    digitizer = Digitizer()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
