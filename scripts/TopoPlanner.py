#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool,String


class robot_path_planner: 
    """
     A Script that plans the route for the robot between the topological nodes, it also starts the
     whole process by moving to start position. This script notifies MoveBaseTopClient which node to move to.
    """


    def __init__(self):
        """
        Publishes to move_to_node topic, subscribed from MoveBaseTopClient Script the String Value of target Node
        Subscribes to topological_feedback, published from both MoveBaseTopClient and ManageWeeding Scripts which 
        notifies when weeding for a node is done (from ManageWeeding) or when target node shouldnt be weeded
        (MoveBaseTopClient) so just move to next node.
        Publishers latched to ensure messages are received by subscribers 
        """

        self.pubToTopClient = rospy.Publisher('/move_to_node',String,latch=True,queue_size=1)
        self.subToTopClientFeedback = rospy.Subscriber('/topological_feedback',Bool,self.moveToNextStage)
        
        self.currentStep = 0

        self.posDict = {10:'WayPoint10',11:'WayPoint11',12:'WayPoint12',13:'WayPoint13',17:'WayPoint17',14:'WayPoint14',15:'WayPoint15',16:'WayPoint16'
        ,18:'WayPoint18',19:'WayPoint19',20:'WayPoint20',21:'WayPoint21',22:'WayPoint22',23:'WayPoint23',24:'WayPoint24'}

        self.plan = [10,11,12,13,17,14,15,16,15,14,17,13,12,11,10,18,19,20,21,22,23,24,23,22,21,20,19,18,10]
        rospy.loginfo("Topo Planner Launched")

    def getNode(self,position):
        node = self.posDict.get(position)
        if node == None:
           
            rospy.loginfo("Key Not Found for %s",position)
        return node
            
    def getPosition(self,currentStep):
        if currentStep < len(self.plan):
            return self.plan[currentStep]


    def startPlan(self):
        """Starts the whole Process"""
        pos = self.getPosition(self.currentStep)
        node = self.getNode(pos)
        self.pubToTopClient.publish(node)
    
    def moveToNextStage(self,msg): 
        """ topological_feedback topic callback Type:Bool
            Receives notification that all weeds (or has meet a target) in area have been dealt with
            Now Publishes the next node in the path to be moved to, to start process again
        """
        self.currentStep +=1
        pos = self.getPosition(self.currentStep)
        node = self.getNode(pos)
        if node is not None:
            self.pubToTopClient.publish(node)
    
rospy.init_node('topologicalPlanner')
tp = robot_path_planner()
tp.startPlan()
rospy.spin()