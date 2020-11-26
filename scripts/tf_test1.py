#!/usr/bin/env python
import rospy
import tf

from geometry_msgs.msg import PoseStamped

from math import atan2, pi


class TFManager:

    def __init__(self):

        self.listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher('test_merl',PoseStamped,queue_size=1)
        self.rate = rospy.Rate(.5)

    def run(self):
        #while not rospy.is_shutdown():
           # try:
            #    (trans,rot) = self.listener.lookupTransform('thorvald_001/base_link','thorvald_001/spray',)
            #except:
             #   pass
        pass
