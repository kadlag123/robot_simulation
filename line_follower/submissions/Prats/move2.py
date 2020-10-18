import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2 as cv

class Listener:
    def __init__(self):
        rospy.init_node('listener')
        self.sub = rospy.Subscriber('/camera/image_raw',Image,self.callback)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
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
            threshold_value=71
            threshold_type=1
            max_binary_value=255
	    pts1 = np.float32([[0,height/2+100],[0,height-40],[width,height/2+100],[width,height-40]])
	    pts2 = np.float32([[0,0],[0,height],[width,0],[width,height]])
            
	    matrix = cv.getPerspectiveTransform(pts1,pts2)


	    result = cv.warpPerspective(frame,matrix,(height,width))

	    frame_gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)

	    _, dst=cv.threshold(frame_gray, threshold_value, max_binary_value, threshold_type)
            rows,cols=dst.shape
            #for i in range(rows):
                #for j in range(rows):
                    #k=dst[i,j]
                    #print k
            cv.circle(frame,(380,680),5,(0,0,255),-1)
            #val=0
            for i in range(rows):
                p=dst[i,420]
                if p==255:
                    k1=i+15
                    print k1
                    break
                    #for l in range(i,rows):
                        #l=dst[l,420]
                        #if l==0:
                            #nonlocal val
                            #val=(i+l)/2
                            #print(val)
                            #break
                    #break   
            #print(val)    
	    #cv.imshow("birds view",result)
            cv.imshow("original",frame)
	    cv.imshow("final_threshold",dst)
            k=cv.waitKey(1)
            #to move bot in forward direction

            self.pub.publish(self.msg)
            self.rate.sleep()
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
