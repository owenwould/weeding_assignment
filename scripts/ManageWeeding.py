#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path



class weed_manager:
    """
     A Script that sits in between the MoveBaseTopClient
     and TopoPlanner and dictates when the weed detector should detect
     weeds and then sends the locations of the weeds to the move base client
     script. Receives feedback from WeedSprayer script to send next weed location
     once all weeds have been moved to the script notifies the TopoPlanner script
     to move to the next node to start the process again. 
    """

    def __init__(self):

        """
        Subscribes to the start_weeding topic, published from MoveBaseTopClient to initiate process
        Publishes to start_detecting topic, subscribed from WeedDetector Script to enable the detector to calculate weed positions
        Subscribes to detected_weed_array, published from WeedDectector Script to receive array of Weed Poses 
        Publishes to move_to_weed, subscribed from MoveBaseClientR Script to send pose of a single weed 
        Subscribes to thorvald_001/sprayerFeedback, published from WeedSprayer Script to get notification when weed has been sprayed 
        Publishes to topological_feedback, subscribed from TopoPlanner to notify when all weeds have been dealt with for particluar node
        Publishers latched to ensure messages are received by subscribers 
        """

        self.subToStartWeeding = rospy.Subscriber('/start_weeding',Bool,self.startWeedingProcess,queue_size=1)
        self.pubToSetDetector = rospy.Publisher('/start_detecting',Bool,queue_size=1,latch=True)
        self.pubTopoFeedback = rospy.Publisher('/topological_feedback',Bool,latch=True,queue_size=1)
        self.subToDetectedWeedArray = rospy.Subscriber('/detected_weed_array',Path,self.getWeedArray,queue_size=1)
        self.pubToMoveBaseClient = rospy.Publisher('/move_to_weed',PoseStamped,latch=True,queue_size=1)
        self.subToSprayerFeedback = rospy.Subscriber('/thorvald_001/sprayerFeedback',Bool,self.receiveSprayerFeedback,queue_size=1)
        self.currentIndex = 0
        self.weedArray = Path().poses
        rospy.loginfo("Manage Weeding Launched")
    
    def startWeedingProcess(self,msg):
        """start_weeding topic callback Type: Bool, enables detector in WeedDetector Script """
        self.pubToSetDetector.publish(msg.data)
        print("Next Weeding Stage")

    def getWeedArray(self,msg):
        """ detected_weed_array topic callback Type: Path (Path has PoseStamped Array field) 
            Array of PoseStamped of weeds, detected by detector
            Begins Moving to individual Weed Process
        """
        print("Received New Weed Array")
        tempArray = Path()
        tempArray.poses = msg.poses
       
        if len(tempArray.poses) == 0 or None or msg.poses == None:
            self.weedArray = tempArray.poses
            print("Finished for node")
            self.pubTopoFeedback.publish(True)
            return

        self.weedArray = tempArray.poses
        self.currentIndex = 0
        self.moveToWeed()
    
    def receiveSprayerFeedback(self,msg):
        """ thorvald_001/sprayerFeedback callback Type: Bool 
            once sprayer has finished, the notification is received by this callback, system 
            can move to the next weed or next node if none left
        """
        self.currentIndex +=1
        if (self.currentIndex < len(self.weedArray)):
            self.moveToWeed() 
        else:
            print("Finished for node")
            self.pubTopoFeedback.publish(True)

    def moveToWeed(self): 
        weed = self.weedArray[self.currentIndex]
        self.pubToMoveBaseClient.publish(weed)
        
rospy.init_node('weedingManager')
wm = weed_manager()
rospy.spin()        
