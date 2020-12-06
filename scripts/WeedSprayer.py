#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Bool

class SprayComponent:
    def __init__(self):
        self.subToSprayCaller = rospy.Subscriber('/thorvald_001/sprayerStart',Bool,self.callSprayer)
        self.serviceName = '/thorvald_001/spray'
    
    def callSprayer(self,msg):
        print("merl")
    
        rospy.wait_for_service(self.serviceName)
        try:
            sprayCall = rospy.ServiceProxy(self.serviceName,Empty)
            respl = sprayCall()
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)


rospy.init_node("sprayManager_" + "001")
roam = SprayComponent()
rospy.spin()