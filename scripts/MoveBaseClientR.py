#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf
from math import atan2,pi

class movebase_client():
    
    """
     A Script that moves the robot to individual weeds using move_base actions, checks if the robot has moved or not due to goal tolerances.
     Once moved to the target weed the WeedSprayer topic thorvald_001/sprayerStart is called to continue the process 
     of weeding the field. 
    """



    def __init__(self):

        """
         Client variable, simple action client for thorvald_001/move_base used to move to weed locations
         Subscribe to move_to_weed, published from MangeWeeding Script to recieve weed PoseStamped to move to
         Publisher to /thorvald_001/sprayerStart, subscribed from WeedSprayer Script used to begin spraying process once at target weed
        """
        self.client = actionlib.SimpleActionClient('/thorvald_001/move_base',MoveBaseAction)
        self.subToWeedingManager = rospy.Subscriber('/move_to_weed',PoseStamped,self.moveToWeeds,queue_size=1)
        self.pubToSprayer = rospy.Publisher('/thorvald_001/sprayerStart',Bool,latch=True,queue_size=1)
        self.rate = rospy.Rate(0.1)
        self.BeforeGoalPosition = PoseStamped()
        self.tfListener = tf.TransformListener()
        rospy.loginfo("MoveBaseClientR Launched")

    def moveToWeeds(self,msg):
        """
         move_to_weed topic callback Type: PoseStamped
         uses action to move to location 
         checks if robot has actually moved and notifies sprayer 
         that robot is in position
        """

        print("New Weed Target")
        self.client.wait_for_server()
        self.setPosition()
        weed = PoseStamped()
        weed = msg
       
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position = weed.pose.position
        goal.target_pose.pose.orientation = weed.pose.orientation
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        
        rospy.loginfo("Goal Position")
        print(goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        results = self.client.get_result()
        canSpray = self.checkDifference()

        self.pubToSprayer.publish(canSpray)
    
    def checkDifference(self):
        """
         Target Goal can fall within move_base's goal tolerance and believe it has reached goal without moving, 
         this function determines if it has moved, if so then spray if not dont spray as this will spray 
         over an area already sprayed. Script also displays difference between goal position and actual 
         position displaying this difference in positioning
        """
        try:
                (trans,rot) = self.tfListener.lookupTransform('/map','/thorvald_001/base_link',rospy.Time(0))
                rospy.loginfo("Actual Position")
                print(trans[0],trans[1])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                self.rate.sleep()
                print(e)
                return 

        
        xDif = abs(trans[0]) - abs(self.BeforeGoalPosition.pose.position.x)
        yDif = abs(trans[1]) - abs(self.BeforeGoalPosition.pose.position.y)
        
        yaw_old = atan2(self.BeforeGoalPosition.pose.orientation.z,self.BeforeGoalPosition.pose.orientation.w) * 2
        yaw_new = atan2(rot[2],rot[3])*2
        yaw_diff = yaw_new - yaw_old
        rospy.loginfo("Yaw Diff %s",yaw_diff)
        if xDif > 1.5:
            #The robot has drifted 
            return False

        if yaw_diff > 0.001 or yaw_diff < -0.001:
            #Has Rotated therefore spray
            return True   

        if xDif > 0.1 or xDif < -0.1: 
            #Has moved 
            
            return True

        elif yDif > 0.1 or yDif < -0.1:
            #Has moved
            return True
        else:
            #Unlikely to have moved so dont spray
            rospy.loginfo("Hasnt Moved")
            return False
    def setPosition(self):
        #Getting latest position before moved to target position
        try:
                (trans,rot) = self.tfListener.lookupTransform('/map','/thorvald_001/base_link',rospy.Time(0))
                self.BeforeGoalPosition.pose.position.x = trans[0]
                self.BeforeGoalPosition.pose.position.y = trans[1]
                self.BeforeGoalPosition.pose.orientation.x = 0
                self.BeforeGoalPosition.pose.orientation.y = 0
                self.BeforeGoalPosition.pose.orientation.z = rot[2]
                self.BeforeGoalPosition.pose.orientation.w = rot[3]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                self.rate.sleep()
                print(e)
                return 

        

rospy.init_node('moveBaseClientR2')
mb = movebase_client()
rospy.spin()
