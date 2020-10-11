import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import Int32
import cv2 as cv
import random as rng

rng.seed(12345)

class Listener:
    def __init__(self):
        rospy.init_node('listener')
        self.sub = rospy.Subscriber('/camera/image_raw',Image,self.callback)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.pub2 = rospy.Publisher('/error',Int32,queue_size=10)
        self.rate = rospy.Rate(1)
        self.msg=Twist()
        self.bridge=CvBridge()
    
    def callback(self,data):
        try:
            frame=self.bridge.imgmsg_to_cv2(data, 'bgr8')
            #self.perform(cv_image)
            #cv.imshow('img_receive',frame)
            width,height,channel=frame.shape
            #print(frame.shape)
            #for i in range(widht
            
            #cv.waitKey(1)

            #k=1
            l_circle=[]
            def sortsecond(val):
               return val[1]
            threshold_value=71
            threshold_type=1
            max_binary_value=255
	    pts1 = np.float32([[0,height/2+100],[0,height-40],[width,height/2+100],[width,height-40]])
	    pts2 = np.float32([[0,0],[0,height],[width,0],[width,height]])
            
	    matrix = cv.getPerspectiveTransform(pts1,pts2)


	    result = cv.warpPerspective(frame,matrix,(height,width))

	    frame_gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)

	    _, dst=cv.threshold(frame_gray, threshold_value, max_binary_value, threshold_type)
            _, contours, _ = cv.findContours(dst, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            
            contours_poly = [None]*len(contours)
            boundRect = [None]*len(contours)
            centers = [None]*len(contours)
            radius = [None]*len(contours)
            for i, c in enumerate(contours):
               contours_poly[i] = cv.approxPolyDP(c, 3, True)
               boundRect[i] = cv.boundingRect(contours_poly[i])
               centers[i], radius[i] = cv.minEnclosingCircle(contours_poly[i])
               l_circle.append((i,radius[i]))
    
            l_circle.sort(reverse=True,key=sortsecond)
            
            
            drawing = np.zeros((dst.shape[0], dst.shape[1], 3), dtype=np.uint8)
            rows,cols,channels=drawing.shape
            cv.circle(drawing,(int(centers[l_circle[0][0]][0]),int(centers[l_circle[0][0]][1])),8,(0,0,255),-1)
            cv.circle(drawing,(int(rows/2),int(cols/2)),8,(0,255,0),-1)
            print("centre of track is",centers[l_circle[0][0]][0],centers[l_circle[0][0]][1])
            print("shift is:",centers[l_circle[0][0]][0]-rows/2)
            #print("countour detected:",len(contours),"\n"); 
              
            for i in range(len(contours)):
                color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
                cv.drawContours(drawing, contours_poly, i, color)
                cv.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), \
                  (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)
                cv.circle(drawing, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)
    
    
            cv.imshow('Contours', drawing)
    
            #for i in range(rows):
                #for j in range(rows):
                    #k=dst[i,j]
                    #print k
            #cv.circle(frame,(380,720),5,(0,0,255),-1)
	    #cv.imshow("birds view",result)
            cv.imshow("original",frame)
	    cv.imshow("final_threshold",dst)
            k=cv.waitKey(1)
            shift=centers[l_circle[0][0]][0]-rows/2
            shift_abs=abs(centers[l_circle[0][0]][0]-rows/2)
            #to move bot in forward direction
            if shift_abs<=50:
                self.msg.linear.x=0.2   #0.15
                self.msg.angular.x=0
            #to rotate the bot
            elif shift<0 and shift_abs<=200:
                self.msg.angular.z=0.75
                self.msg.linear.x=0.025
                rospy.Duration(2)
            elif shift>0 and shift_abs<=200:
                self.msg.angular.z=-0.1
                self.msg.linear.x=0.025
                rospy.Duration(2)
            elif shift_abs>=203:
                self.msg.angular.z=0
                self.msg.linear.x=-0.1  #had changed values earlier         
            elif k==ord('q'):
                rospy.signal_shutdown("shutdown")
                cv.destroyAllWindows()
            else:
                self.msg.angular.z=0
                self.msg.linear.x=0  
            #self.pub.publish(self.msg)
            self.pub2.publish(int(shift))
            #self.rate.sleep()
        except CvBridgeError as e:
            print(e)

    def perform(self,img):
        cv.imshow('img_receive',img)
        
           
if __name__ == '__main__':
    listener=Listener()
    try:
        #Testing our function
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print(e)
