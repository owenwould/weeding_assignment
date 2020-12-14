#!/usr/bin/env python
import rospy 
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped

class TFBroadcastComp:
    def __init__(self,frame):
        self.tfListener = tf.TransformListener()
        #self.sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.getRobotPosition,queue_size=1)
        #self.pubRobotPose = rospy.Publisher(frame + '/tf_broadcast',PoseStamped,queue_size=1)
        self.robotFrame = frame
        self.sub = rospy.Subscriber('/thorvald_001/scan',LaserScan,self.getRobotPosition)
        rospy.loginfo("BroadcastTf Launched")
    
    def getRobotPosition(self,msg):
        #self.tfListener.waitForTransform('/thorvald_001/base_link','/thorvald_001/kinect2_camera/hd/image_color_rect')
        #self.tfListener.waitForTransform('/thorvald_001/base_link','/thorvald_001/kinect2_camera/hd/image_color_rect')
        
        
        
        pose_base = PoseStamped()
        
       
        
        
   
if __name__ == '__main__':
    rospy.init_node('tfMapToRobot')
    mapToBase = TFBroadcastComp('/thorvald_001/base_link')
    rospy.spin()
            

            

