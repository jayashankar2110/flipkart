#!/usr/bin/env python
'''
goal aborted then change status to idle. Can be done by 
checking status code of result
'''
import rospy
import actionlib
from single_bot.msg import state1Action
from single_bot.msg import state1Goal
from std_msgs.msg import String

simulate = False

class state1Client:

    def __init__(self):
        self.goal_sent = False
        self._ac = actionlib.SimpleActionClient('/Navigation',state1Action)
        self._ac.wait_for_server()
        #self._pub = rospy.Publisher('/state1Status',String,queue_size=1)
        rospy.loginfo("Navigation server is up")

    def send_nav_goal(self,goal):
        self._ac.send_goal(goal,done_cb=self.done_callback,feedback_cb=self.feedback_callback)
        rospy.loginfo("Goal has been sent")
        self.goal_sent = True
        #rospy.loginfo(self._ac.get_result())
        #pub.publish(result.Reached)

    def done_callback(self,status,result):
        #rospy.loginfo("Success status is: ", status)
        #rospy.loginfo("Success result is: ",result)
        if result:
            #self._pub.publish(result.Reached)
            rospy.sleep(1.0)
            rospy.loginfo('goal Reached')
            self.goal_sent = False
            f_state = rospy.get_param('f_state')
            rospy.set_param('c_state', f_state)    # switch to unloding state
        else:
            rospy.logwarn("None type results reached to client")
    
    def feedback_callback(self,feedback):
        rospy.loginfo(feedback.deviation)
        #if feedback.deviation > 0.15:
        #    rospy.loginfo("aborting tracking path as bot is deviating")
        #    self.abortGoal()

    def cancelGoal(self):
        rospy.sleep(2)
        self._ac.cancel_goal()
        rospy.logwarn('Goal canceled')

def callback(msg,client):
    if msg.data == 'navigate':

        i_state = rospy.get_param('i_state')
        g_state = rospy.get_param('f_state')
        T = rospy.get_param('duration')  # max simulation time
        rospy.loginfo(msg)
        action_msg = state1Goal()
        action_msg.targetPose = g_state
        action_msg.currentPose = i_state
        action_msg.tFinal = T
        client.send_nav_goal(action_msg)
    
    if msg.data == 'Idle' and client.goal_sent == True:
        client.cancelGoal()
        client.goal_sent = False
    if msg.data == 'hold':
        #keep pending status.. (need to develop)
        client.cancelGoal()
        pass

def sim_callback(client):
    i_state = rospy.get_param('i_state')
    g_state = rospy.get_param('f_state')
    T = rospy.get_param('duration')  # max simulation time
    action_msg = state1Goal()
    action_msg.targetPose = g_state
    action_msg.currentPose = i_state
    action_msg.tFinal = T
    client.send_nav_goal(action_msg)


if __name__ == '__main__':
    rospy.init_node('nav_Client')
    client = state1Client()
    if not simulate:
        rospy.Subscriber('/state_id', String,callback,client)
    if simulate:
        sim_callback(client)
    rospy.spin()