#!/usr/bin/env python
import rospy 
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String, Bool
class movebase_top_client:

    """
     A Script that moves the robot along the topological nodes using actions, provides feedback when 
     node has sucessfully moved to the target node. Provides the heading direction i.e. 
     moving down or up the rows to inform calculation of weed position for the WeedDetector Script to overcome issues
     because of positive and negative coordinate system. 
    """

    def __init__(self):

        """
         Client variable simple action client for /thorvald_001/topological_navigation used to move to node
         Subscribes to move_to_node, published from TopoPlanner Script to receive target node String name to move to
         Publishes to start_weeding, subscribed from ManageWeeding Script to start weeding process at node 
         Publishes to robot_moving_down, subscribed from WeedDetector to set whether robot is moving down crop or not
         Publishes to topological_feedback, subscribed from TopoPlanner to notify that the robot should move to the next
         node in the plan. This is done for nodes which aren't located on the crops. 
         Publishers latched to ensure messages are received by subscribers
        """
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation',GotoNodeAction)
        self.subToMoveRobot = rospy.Subscriber('/move_to_node',String,self.setNextGoal)
        self.pubToStartWeeding = rospy.Publisher('/start_weeding',Bool,latch=True,queue_size=1)
        self.pubRobotMovingDown = rospy.Publisher('/robot_moving_down',Bool,latch=True,queue_size=1)
        self.pubToTopFeedback = rospy.Publisher('/topological_feedback',Bool,latch=True,queue_size=1)
        self.lastNode = None
        self.isMovingDown = False

        self.nodesDict = {'WayPoint10':False,'WayPoint11':True,'WayPoint12':True,'WayPoint13': True,'WayPoint17':False,'WayPoint14':True,'WayPoint15':True,'WayPoint16':False
        ,'WayPoint18':True,'WayPoint19':True,'WayPoint20':True,'WayPoint21':False,'WayPoint22':True,'WayPoint23':True,'WayPoint24':False}
        
        self.positionDict = {'WayPoint10':8,'WayPoint11':5,'WayPoint12':0,'WayPoint13': -5,'WayPoint17': -8,'WayPoint14': -5,'WayPoint15':0,'WayPoint16':5
        ,'WayPoint18':5,'WayPoint19':0,'WayPoint20':-5,'WayPoint21':-8,'WayPoint22':-5,'WayPoint23':0,'WayPoint24':5}

        self.dontStopNodes = ['WayPoint10','WayPoint21','WayPoint17']
        self.endNodes = ["WayPoint16","WayPoint24"]
        rospy.loginfo("MoveBaseTopClient Launched")
    
    def setNextGoal(self,msg):
        """ 
         move_to_node topic callback Type: String 
         Uses Topological Action client to move to target node, orientation of goal is decided and moving down status as well
         Weeding process is begun once robot reaches goal if node is a weeding node 
        """
        
        print("Next Topological Goal")
        self.client.wait_for_server()
        targetNode = msg.data
        goal = GotoNodeGoal()
        goal.target = targetNode

        if self.lastNode is not None:
            movingDownStatus = self.getMovingDownDir(targetNode)
            self.pubRobotMovingDown.publish(movingDownStatus)

        if targetNode in self.endNodes:
            #Always Moving down from end nodes
            self.pubRobotMovingDown.publish(True)

        orientationState = self.getOrientationStatus(targetNode)
        goal.no_orientation = orientationState
        self.client.send_goal(goal)
        status = self.client.wait_for_result()
        result = self.client.get_result()
        self.lastNode = targetNode
        
        if targetNode in self.dontStopNodes:
            #Dont stop and weed, move straight to next node
            self.pubToTopFeedback.publish(True)
        else:
            self.pubToStartWeeding.publish(True)
        

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
        """
         Calculates whether robot is moving down the cropped field or not
         to inform calculation of position of weeds 
        """
        oldVal = self.getNodePos(self.lastNode)
        newVal = self.getNodePos(newNode)

        if oldVal > newVal:
            movingDown = True
        else:
            movingDown = False
        self.isMovingDown = movingDown
        return movingDown

rospy.init_node('topologicalMoveBaseClient')
mc = movebase_top_client()
rospy.spin()





        
