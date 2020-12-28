import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal



class movebase_client():
  
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        

    def setGoal(self):
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 4.14984268563
        goal.target_pose.pose.position.y = -2.07400995865
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0    
        goal.target_pose.pose.orientation.z = 0.999957010129
        goal.target_pose.pose.orientation.w = -0.00927242656581

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()


        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()  


if __name__ == '__main__':
    try:
        rospy.init_node('memerser')
        m = movebase_client()
        r = m.setGoal()
        if r:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


