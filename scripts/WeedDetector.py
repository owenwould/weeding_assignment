#!/usr/bin/env python
import rospy 
import cv2
import numpy as np
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from cv_bridge import CvBridge
import sys

class DetectorComponent:
    def __init__(self,id,colourID,limits):
        self.subKinectCam = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/image_color_rect',Image,self.detectGround,queue_size=1)
        self.bridge = CvBridge()
        self.pubFilteredImg = rospy.Publisher('/thorvald_' + id + '/' + colourID,Image,queue_size=1)
        self.colour = colourID
        self.upperLimit = limits[0]
        self.lowerLimit = limits[1]

        self.subToBoundingBox = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.getMerl,queue_size=1)


    
    def detectGround(self,msg):
       
        image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)


        #HSV
        mask = cv2.inRange(image,self.lowerLimit,self.upperLimit)
        image = cv2.bitwise_and(image,image,mask=mask)
        self.pubFilteredImg.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(image,cv2.COLOR_HSV2BGR),"bgr8"))
    
    def getMerl(self,msg):
        x = msg.bounding_boxes



        for y in x:
            
            if y.probability > 0.5:

                xCenter = (y.xmin + y.xmax) / 2
                yCenter = (y.ymin + y.ymax) / 2
                print(xCenter,yCenter)

        
    



        



thorvaldID = str(sys.argv[1]) if len(sys.argv) > 1 else "001"

rospy.init_node('detector_' + thorvaldID,anonymous=True)

upperLimit = np.array([40,0,20])
lowerLimit = np.array([80,255,255])

limit = np.array([lowerLimit,upperLimit])
detector = DetectorComponent(thorvaldID,"letticeGreen",limit)



rospy.spin()


    
        
        











