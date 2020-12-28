#!/usr/bin/env python
import rospy 
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String, Bool
class movebase_top_client:

    def __init__(self):
        rospy.loginfo("Start")
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation',GotoNodeAction)
        self.subToMoveRobot = rospy.Subscriber('/move_to_node',String,self.setNextGoal)
        self.pubTopoFeedback = rospy.Publisher('/topological_feedback',Bool,latch=True,queue_size=1)
        self.pubRobotMovingDown = rospy.Publisher('/robot_moving_down',Bool,latch=True,queue_size=1)
        self.rate = rospy.Rate(0.2)
        self.lastNode = None

        self.nodesDict = {'WayPoint10':False,'WayPoint11':True,'WayPoint12':True,'WayPoint13': True,'WayPoint17':False,'WayPoint14':True,'WayPoint15':True,'WayPoint16':False
        ,'WayPoint18':True,'WayPoint19':True,'WayPoint20':True,'WayPoint21':False,'WayPoint22':True,'WayPoint23':True,'WayPoint24':False}
        
        self.positionDict = {'WayPoint10':8,'WayPoint11':5,'WayPoint12':0,'WayPoint13': -5,'WayPoint17': -8,'WayPoint14': -5,'WayPoint15':0,'WayPoint16':5
        ,'WayPoint18':5,'WayPoint19':0,'WayPoint20':-5,'WayPoint21':-8,'WayPoint22':-5,'WayPoint23':0,'WayPoint24':5}
    
    def setNextGoal(self,msg):
        self.client.wait_for_server()
        targetNode = msg.data
        goal = GotoNodeGoal()
        goal.target = targetNode

        if self.lastNode is not None:
            movingDownStatus = self.getMovingDownDir(targetNode)
            self.pubRobotMovingDown.publish(movingDownStatus)

        orientationState = self.getOrientationStatus(targetNode)
        goal.no_orientation = orientationState
        
        self.client.send_goal(goal)
        status = self.client.wait_for_result()
        result = self.client.get_result()
        self.lastNode = targetNode
        self.rate.sleep()
        self.pubTopoFeedback.publish(True)
    

    def getOrientationStatus(self,node):
        state = self.nodesDict.get(node)
        if state == None:
            rospy.loginfo("Error with orientation status for node in MoveBaseTopClient: %s",node)
        return state
    def getNodePos(self,node):
        val = self.positionDict.get(node)
        if val == None:
            rospy.loginfo("Error with position for node in MoveBaseTopClient: %s",node)
        return val

    
    def getMovingDownDir(self,newNode):
        oldVal = self.getNodePos(self.lastNode)
        newVal = self.getNodePos(newNode)
        movingDown = True

        if oldVal > newVal:
            movingDown = True
        else:
            movingDown = False
        
        return movingDown

  



   
rospy.init_node('topologicalMoveBaseClient')
mc = movebase_top_client()
rospy.spin()





        
