#!/usr/bin/env python
import rospy 
from std_msgs.msg import Bool, String 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import sys

class RoamerComponent:
    def __init__(self,robotID):
        #Sub to topic that stops robot 
        #publish to cmd/Vel 
        self.subToStop = rospy.Subscriber('/thorvald_001/roamerStop',Bool,self.stopRoamer)
        self.RoamerDisabled = False
        self.RoamerMovePub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel',Twist,queue_size=0)
        self.forwardSpeed = 2.0
        self.obstacleDetectionSub = rospy.Subscriber('/thorvald_001/scan',LaserScan,self.detectObstacle)
        self.sprayActive = False
        self.SprayerPub = rospy.Publisher('/thorvald_001/sprayerStart',Bool,queue_size=0) 




    def stopRoamer(self,msg):
        self.RoamerDisabled = msg.data

    def detectObstacle(self,msg):
        self.moveRobot()




    def moveRobot(self):
       
        t = Twist()
        t.linear.x = 0.1
        self.RoamerMovePub.publish(t)
       
       # self.SprayerPub.publish(False)
       
    



id = str(sys.argv[1]) if len(sys.argv) > 1 else "001"

rospy.init_node("roamer_" + id)
roamerX = RoamerComponent(id)

rospy.spin()


        

        


          