#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Bool

class SprayComponent:
    """
     A Script that handles the sprayer action (calls the service), notifies the ManageWeed script
     when the spray has finished. Receives notification from MoveBaseClientR script when to spray. 
    """
    def __init__(self):
        """
         Subscriber to thorvald_001/sprayerStart, published from MoveBaseClientR to notify when robot above a weed
         Publishes to thorvald_001/sprayerFeedback, subscribed to by ManageWeeding to notify when target weed has been sprayed
        """

        self.subToSprayCaller = rospy.Subscriber('/thorvald_001/sprayerStart',Bool,self.callSprayer)
        self.pubToWeedingManager = rospy.Publisher('/thorvald_001/sprayerFeedback',Bool,latch=True,queue_size=1)
        self.serviceName = '/thorvald_001/spray'
        self.sprayCount = 0
        self.failCount = 0
        rospy.loginfo("Spayer Launched")
    
    def callSprayer(self,msg):
        """ 
            thorvald_001/sprayerStart topic callback Type: Bool 
            spray weed if msg = True, based off robot has moved from MoveBaseClientR. 
            Goal Tolerance can mean robot has completed goal without actually moving so 
            dont spray but still notify Weed Manager that action is complete so next weed can be dealt with
        """
        if msg.data == False:
            self.pubToWeedingManager.publish(True)
            self.failCount +=1
            print("fail Count ", self.failCount)
            rospy.loginfo("Fail Count: %s",self.failCount)
            return


        rospy.wait_for_service(self.serviceName)
        try:
            sprayCall = rospy.ServiceProxy(self.serviceName,Empty)
            respl = sprayCall()
            self.sprayCount +=1
            print("Success Rate ", self.sprayCount)
            rospy.loginfo("Success Count: %s",self.sprayCount)
            self.pubToWeedingManager.publish(True)

        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    
rospy.init_node("sprayManager")
roam = SprayComponent()
rospy.spin()