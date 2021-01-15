#!/usr/bin/env python
import rospy
import cv2 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#!!Script used to capture images for yolo annotation not part of actual solution
class capture_image:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.imageSub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect',Image,self.displayImage)
        self.num =0

    def displayImage(self,msg):
        cv2.namedWindow("Image window")
        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        cv2.imshow("Image window",cv_image)
       

        path = 'cap' + str(self.num) + ".jpg"
        cv2.imwrite(path,cv_image)
        self.num +=1

        cv2.waitKey(1)

rospy.init_node("image_cap")
ic = capture_image()
rospy.spin()