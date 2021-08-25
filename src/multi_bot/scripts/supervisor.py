#!/usr/bin/env python

"""
STATE identification MOVED TO FEEDBACK NODE. Can rethink about this arrangement.

Node named central_monitor subscribe to '/feedback' topic for c_state, downloads bot parameters - f_state and i_state
publish the state identifier to topic /state_id

state_ids are [start, navigate, unload, return, stop]

"""
import rospy
from flipkart.msg import localizemsg
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(msg)

    return

def talker():
    rospy.init_node('CentralMonitor', anonymous= True)
    pub = rospy.Subscriber('/state_id',String,callback)
    #sub = rospy.Subscriber('/feedback',localizemsg,callback,pub)
    rospy.spin()

if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass