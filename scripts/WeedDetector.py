#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path,Odometry
import image_geometry
from std_msgs.msg import Bool
import tf

class DetectorComponent:

    """
     A Script that detects weeds when enabled and calculates the world position of the weeds detected. The weeds positions
     are stored in an array before being passed to detected_weed_array topic to be dealt with. Detection system is a 
     YOLOv3 - Tiny Custom Detector with 3 classes Weed, Broccoli and Young Broccoli with a threshold set to 0.3. 
     Note ros.yaml from darknet_ros has camera reading topic set to /thorvald_001/kinect2_camera/hd/image_color_rect
    """

    def __init__(self):
        
        """
         Subscribes to darknet_ros/bounding_boxes which provides an array of bounding box for all the detected objects, 
         used to differentiate objects to identify weeds and subsuqently calculate their world locations.
         Subscribes to thorvald_001/kinect2_camera/hd/camera_info to get camera informaion nessecary for using image_geometry's
         PinHole Camera Model for calculating position of weed
         Subscribes to robot_moving_down, published from MoveBaseTopClient to account for positive and negative coordinate system in weed
         position calculation
         Subscribes to start_detecting, published from ManageWeeding to enable custom detector to calculate weed locations
         Publishes to detected_weed_array, subscribed from ManageWeeding to send array of weed poses 
         Publishers latched to ensure messages are received by subscribers
        """


        self.subToBoundingBox = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.detectObjects,queue_size=1)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info',CameraInfo,self.setCameraModel)
        self.subToMovingDown = rospy.Subscriber('/robot_moving_down',Bool,self.setDirection,queue_size=1)
        self.subToStartDetecting = rospy.Subscriber('/start_detecting',Bool,self.setCanDetect,queue_size=1)
        self.pubDetectedWeedArray = rospy.Publisher('/detected_weed_array',Path,latch=True,queue_size=1)
        self.camera_model = None 
        self.movingDown = True
        self.canDetect = False #Default False
        self.depth = 0.5
        self.tfListener = tf.TransformListener()
        rospy.loginfo("WeedDetector Launched")

   

    def setCanDetect(self,msg):
        """ start_detecting topic callback Type:Bool, enables custom detector"""
        self.canDetect = msg.data

    def setDirection(self,msg):
        """ robot_moving_down topic callback Type:Bool, sets moving down status"""
        self.movingDown = msg.data
    
  
    
    def detectObjects(self,msg):
        """ 
         darknet_ros/bounding_boxes topic callback Type: Bounding Boxes  
         When enabled (i.e self.canDetect) used to get locations of weeds for a single msg, once positions 
         calculated for first scan detector disabled to prevent duplications of same weeds being stored in 
         array. Depth value is constant and is 0.5 of a metre
        """

        
        if self.camera_model == None or not self.canDetect:   
            return

        canSend = False

        detected_objs = msg.bounding_boxes

        """ 
         detected_objs is a list of bounding boxes from msg, used to loop through and act on weed objects. 
         Base_link position of robot is calculated relative to map fame to get current world positon of robot
        """

        try:
                (trans,rot) = self.tfListener.lookupTransform('/map','/thorvald_001/base_link',rospy.Time(0))
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
                return 
        
        newSet = Path()
    

        for box in detected_objs:
            

            if box.probability > 0.5 and box.id == 0:
                
                #id 0 = Weed, id 1 = Broccoli and id 2 = YoungBroccoli
                #Note Detector threshold is set to 0.3 (in config) but only objects with a probability of greater than 0.5 are considered

                """ 
                 Calculate the world position of the weed
                 First get pixel coordinates using bounding box corner positions
                 Then Calculate the world position relative to the camera using project ray and the known depth, as 
                 ray returns a series of different positions along a ray with the depth being used to get the estimated location
                 of the weed. 
                """
                
                u = (box.xmin + box.xmax) / 2 
                v = (box.ymin + box.ymax) / 2
                ray = self.camera_model.projectPixelTo3dRay((u,v))
                pt = [el * self.depth for el in ray] 
        
                p_pose = PoseStamped()
                p_pose.header.frame_id = '/thorvald_001/kinect2_rgb_optical_frame'
                
                p_pose.pose.position.x = pt[0]
                p_pose.pose.position.y = pt[1]
                p_pose.pose.position.z = pt[2]

                #Alternative Way to calculate weed position, there isnt much difference between values using either approach
                #aX = ((u-self.camera_model.cx())/self.camera_model.fx()) * self.depth
                #aY = ((v-self.camera_model.cy())/self.camera_model.fy()) * self.depth
                #aZ = self.depth
                #p_pose.pose.position.x = aX
                #p_pose.pose.position.y = aY
                #p_pose.pose.position.z = aZ

                """ 
                 Transform Camera coordinates into sprayer coordinates 
                 Offset this value against the world location of the base_link (move_base moves using base_link position relative to map)
                 depending on whether robot is moving down or up row (positive and negative coordinate system)
                 Finally add the world location of the weed relative to the map to weed location array
                """

                p_spray = self.tfListener.transformPose('/thorvald_001/sprayer',p_pose)
                p_robot_base = PoseStamped()
                p_robot_base.pose.position.x = trans[0] 
                p_robot_base.pose.position.y = trans[1]

                if self.movingDown:
                    p_robot_base.pose.position.x -= abs(p_spray.pose.position.x) 
                    p_robot_base.pose.position.y += p_spray.pose.position.y
                   
                else:
                    p_robot_base.pose.position.x += abs(p_spray.pose.position.x) 
                    p_robot_base.pose.position.y += p_spray.pose.position.y
                    
                p_robot_base.pose.orientation.x = 0 
                p_robot_base.pose.orientation.y = 0
                p_robot_base.pose.orientation.z = rot[2]
                p_robot_base.pose.orientation.w = rot[3] 
                p_robot_base.header.frame_id = 'map'

                if len(newSet.poses) < 5: 
                    #Target set to 5 but can be higher without any issues justs takes longer to complete
                    newSet.poses.append(p_robot_base)
                    
        canSend = True #Currently not required but could be built upon in future
        

        if canSend:
            """ 
             Once first scan is complete disable detector, to prevent duplications of the same weed 
             just in a slighly different location. Sort the weed array so weeds follow on for each other
             preventing a target weed from being behind the robot which can cause the robot to drift and fail to reach
             target.
            """

            print("Sending New Weed Array")
        
            self.canDetect = False
            
            #sort list https://stackoverflow.com/questions/403421/how-to-sort-a-list-of-objects-based-on-an-attribute-of-the-objects

            if self.movingDown:
                newSet.poses.sort(key=lambda x: x.pose.position.x,reverse=True)
            else:
                newSet.poses.sort(key=lambda x: x.pose.position.x,reverse=False)
            
            #Pass Poses array to manager 
            newSet.header.frame_id = 'map'
            self.weedCoordinateArray = newSet.poses
            self.pubDetectedWeedArray.publish(newSet)
                
                    
        
    def setCameraModel(self,msg):
        """ 
         thorvald_001/kinect2_camera/hd/camera_info topic callback Type: CameraInfo
         Used to get camera characteristics for PinHoleCameraModel then unsubscribes as not needed once values 
         have been set.
        """
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_sub.unregister()


rospy.init_node('weedDetector')
detector = DetectorComponent()
rospy.spin()









    
        
        











