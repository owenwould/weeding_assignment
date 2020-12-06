#!/usr/bin/env python
import rospy 
import cv2
import numpy as np
from sensor_msgs.msg import Image,CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import image_geometry
from cv_bridge import CvBridge
import sys
import tf

class DetectorComponent:
    def __init__(self,id,colourID,limits):
        #self.subKinectCam = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/image_color_rect',Image,self.detectGround,queue_size=1)
        self.bridge = CvBridge()
       # self.pubFilteredImg = rospy.Publisher('/thorvald_' + id + '/' + colourID,Image,queue_size=1)
        self.colour = colourID
        self.upperLimit = limits[0]
        self.lowerLimit = limits[1]
        self.subToBoundingBox = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.detectObjects,queue_size=1)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info',CameraInfo,self.setCameraModel)
        self.subToBroadcastRobPose = rospy.Subscriber('/thorvald_001/base_link/tf_broadcast',PoseStamped,self.setCurrentPose,queue_size=1)
        self.camera_model = None 
        self.depth = 0.5
        self.tfListener = tf.TransformListener()
        self.currentBasePose = PoseStamped()
        self.weedCoordinateArray = Path() #Has PoseStampedArray 
        self.test = False

    def setCurrentPose(self,msg):
        self.currentBasePose = msg
        print("Message")
        rospy.loginfo("Current Pose:")
        rospy.loginfo(self.currentBasePose)



    
    def detectGround(self,msg):
       
        image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        #HSV
        mask = cv2.inRange(image,self.lowerLimit,self.upperLimit)
        image = cv2.bitwise_and(image,image,mask=mask)
        #self.pubFilteredImg.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(image,cv2.COLOR_HSV2BGR),"bgr8"))
    
    def detectObjects(self,msg):
        
        if self.camera_model == None:
            return

        detected_objs = msg.bounding_boxes

        for box in detected_objs:
            

            if box.probability > 0.5:
                

                
                u = (box.xmin + box.xmax) / 2
                v = (box.ymin + box.ymax) / 2
                #ray = self.camera_model.projectPixelTo3dRay((u,v))
                #pt = [el * self.depth for el in ray] 
                #print(pt)


                aX = ((u-self.camera_model.cx())/self.camera_model.fx()) * self.depth
                aY = ((v-self.camera_model.cy())/self.camera_model.fy()) * self.depth
                aZ = 1 * self.depth


                p_pose = PoseStamped()
                p_pose.header.frame_id = 'thorvald_001/kinect2_rgb_optical_frame'
                #p_pose.pose.position.x = pt[0]
                #p_pose.pose.position.y = pt[1]
                #p_pose.pose.position.z = pt[2]

                p_pose.pose.position.x = aX
                p_pose.pose.position.y = aY

                p_spray = self.tfListener.transformPose('thorvald_001/sprayer',p_pose)
                p_robot_base = self.tfListener.transformPose('thorvald_001/base_link',p_spray)

               
                p_robot_base.pose.position.x += self.currentBasePose.pose.position.x
                p_robot_base.pose.position.y += self.currentBasePose.pose.position.y

               
                if self.test == False:
                    print(self.currentBasePose)
                    self.weedCoordinateArray.poses.append(p_robot_base)
                    print(p_robot_base)
                    self.test = True






                

              

                #print(aX,aY)
                


            

    def setCameraModel(self,msg):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_sub.unregister()


    



        



thorvaldID = str(sys.argv[1]) if len(sys.argv) > 1 else "001"

rospy.init_node('detector_' + thorvaldID,anonymous=True)

upperLimit = np.array([40,0,20])
lowerLimit = np.array([80,255,255])

limit = np.array([lowerLimit,upperLimit])
detector = DetectorComponent(thorvaldID,"letticeGreen",limit)



rospy.spin()


    
        
        











