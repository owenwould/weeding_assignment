#!/usr/bin/env python
import rospy 
import tf
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped

class TFBroadcastComp:
    def __init__(self,frame):
        self.tfListener = tf.TransformListener()
        self.sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.getRobotPosition,queue_size=1)
        self.pubRobotPose = rospy.Publisher(frame + '/tf_broadcast',PoseStamped,queue_size=1)
        self.robotFrame = frame
    
    def getRobotPosition(self,msg):
        pose_base = PoseStamped()
        
        pose_base.header.frame_id = self.robotFrame
        pose_base.pose = msg.pose.pose
        print(pose_base)
        self.pubRobotPose.publish(pose_base)

       
if __name__ == '__main__':
    rospy.init_node('tfMapToRobot')
    mapToBase = TFBroadcastComp('/thorvald_001/base_link')
    rospy.spin()
            

            

