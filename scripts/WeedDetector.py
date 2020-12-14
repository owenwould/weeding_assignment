#!/usr/bin/env python
import rospy 
import cv2
import numpy as np
from sensor_msgs.msg import Image,CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,PoseArray
from nav_msgs.msg import Path,Odometry
import image_geometry
from cv_bridge import CvBridge
import sys
import tf

class DetectorComponent:
    def __init__(self):
        #self.subKinectCam = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/image_color_rect',Image,self.detectGround,queue_size=1)
        self.bridge = CvBridge()
        #self.pubFilteredImg = rospy.Publisher('/thorvald_' + id + '/' + colourID,Image,queue_size=1)
      
        self.subToBoundingBox = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.detectObjects,queue_size=1)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info',CameraInfo,self.setCameraModel)
        self.subToAMCL = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.setCurrentPose,queue_size=1)
        self.camera_model = None 
        self.movingDown = False
        self.canDetect = False
        self.currentPoseSet = False
        self.depth = 0.5
        self.subToOdem = rospy.Subscriber('/thorvald_001/odometry/base_raw',Odometry,self.getInitialPoseOrientation)
        self.tfListener = tf.TransformListener()
        self.currentBasePose = PoseStamped()
        self.weedCoordinateArray = Path()
        self.weedCoordinateArray2 = Path() #Has PoseStampedArray 
        self.test = False
        self.rate = rospy.Rate(.5)
        rospy.loginfo("WeedDetector Launched")

    def setCurrentPose(self,msg):    
        
        pos =  PoseStamped()
        pos.header = msg.header
        pos.pose = msg.pose.pose
        if self.currentPoseSet:
            self.setDirection(self.currentBasePose.pose.position,pos.pose.position)
        
        self.currentBasePose = pos
        if not self.currentPoseSet:
            self.currentPoseSet = True
        
        pub = rospy.Publisher('/merl',PoseStamped,queue_size=1)
        pub.publish(self.currentBasePose)


    
    def displayWeedLocation(self):
        while not rospy.is_shutdown():
            pub = rospy.Publisher('/ememem',PoseStamped,queue_size=1)
            weedArray = self.weedCoordinateArray.poses
        
            self.displaySpottedLocation()
            for weed in weedArray:
                pub.publish(weed)
                #print(weed)

            self.rate.sleep()
    

    def displaySpottedLocation(self):
        #print("X")
        #print(len(self.weedCoordinateArray2.poses))
         
        pub = rospy.Publisher('/spot22',PoseStamped,queue_size=1)
        weedArray2 = self.weedCoordinateArray2.poses
        
        for x in weedArray2:
            pub.publish(x)
                

          

    def setDirection(self,oldPose,newPose):
        dir = newPose.x - oldPose.x
        if dir < 0: 
            #moving down
            self.movingDown = True
        else:
            self.movingDown = False
            #moving up 

    
    def detectGround(self,msg):
       
        image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        #HSV
       # mask = cv2.inRange(image,self.lowerLimit,self.upperLimit)
        #image = cv2.bitwise_and(image,image,mask=mask)
        #self.pubFilteredImg.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(image,cv2.COLOR_HSV2BGR),"bgr8"))
    
    def detectObjects(self,msg):
        
        if self.camera_model == None or not self.canDetect:
            return

        detected_objs = msg.bounding_boxes

        

        for box in detected_objs:
            

            if box.probability > 0.33:
                
                u = (box.xmin + box.xmax) / 2
                v = (box.ymin + box.ymax) / 2
                ray = self.camera_model.projectPixelTo3dRay((u,v))
                pt = [el * self.depth for el in ray] 
                #print(pt)


                #aX = ((u-self.camera_model.cx())/self.camera_model.fx()) * self.depth
                #aY = ((v-self.camera_model.cy())/self.camera_model.fy()) * self.depth
                #aZ = 1 * self.depth

                p_pose = PoseStamped()
                p_pose.header.frame_id = '/thorvald_001/kinect2_rgb_optical_frame'
                
                p_pose.pose.position.x = pt[0]
                p_pose.pose.position.y = pt[1]
                p_pose.pose.position.z = pt[2]

                
                #p_pose.pose.position.x = aX
                #p_pose.pose.position.y = aY
                #p_pose.pose.position.z = aZ


                #P2_pose = self.tfListener.transformPose('/thorvald_001/base_link',p_pose)
                #P2_pose.pose.position.x = self.currentBasePose.pose.position.x
                #P2_pose.pose.position.y = self.currentBasePose.pose.position.y
               # P2_pose.pose.orientation = self.currentBasePose.pose.orientation

                #P2_pose.header.frame_id = '/map'

                #p_base = self.tfListener.transformPose('/thorvald_001/base_link',p_pose)
                #p_robot_base = PoseStamped()
                #p_robot_base.header.frame_id = '/thorvald_001/base_link'
                #p_robot_base.pose.position.x = self.currentBasePose.pose.position.x + p_base.pose.position.x
                #p_robot_base.pose.position.y = self.currentBasePose.pose.position.y + p_base.pose.position.y
                #p_robot_base.pose.orientation = self.currentBasePose.pose.orientation
                #p_robot_base.header.frame_id = '/map'




                p_spray = self.tfListener.transformPose('/thorvald_001/sprayer',p_pose)
                p_robot_base = PoseStamped()
                p_robot_base.pose.position.x = self.currentBasePose.pose.position.x
                p_robot_base.pose.position.y = self.currentBasePose.pose.position.y

                #For Moving up
                #p_robot_base.pose.position.x += p_spray.pose.position.x 

                P_x = self.tfListener.transformPose('/thorvald_001/base_link',p_pose)
               

           
                p_robot_base.pose.position.x -= p_spray.pose.position.x 
                p_robot_base.pose.position.y -= p_spray.pose.position.y
                p_robot_base.pose.orientation = self.currentBasePose.pose.orientation
                p_robot_base.header.frame_id = '/map'

                
               
                if self.test == False:
                    self.weedCoordinateArray.poses.append(p_robot_base)
                    #self.weedCoordinateArray2.poses.append(P2_pose)
                    self.test = True
                break
                    
                    


            

    def setCameraModel(self,msg):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_sub.unregister()


    

    def getInitialPoseOrientation(self,msg):
        odem = Odometry()
        odem.pose.pose.orientation = msg.pose.pose.orientation
        self.canDetect = True
        self.subToOdem.unregister()


        



thorvaldID = str(sys.argv[1]) if len(sys.argv) > 1 else "001"

rospy.init_node('detector_' + thorvaldID,anonymous=True)

detector = DetectorComponent()
detector.displayWeedLocation()







    
        
        











