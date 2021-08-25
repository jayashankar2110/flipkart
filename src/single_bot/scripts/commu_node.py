#!/usr/bin/env python

"""
takes message in format 
float32 vr
float32 vl
bool ifUnload
bool isLoaded
and send to bot.
"""

import rospy
from single_bot.msg import com_msg

def callback(msg,pub):
    rospy.loginfo(msg)
    if msg.ifUnload and msg.isLoaded:
        # do unload
        rospy.sleep(1)
        msg.isLoaded = False
        pub.publish(msg)
    return

def talker():
    rospy.init_node('commu_node')
    rospy.Rate(2)
    pub = rospy.Publisher('/commu', com_msg,queue_size=1)
    rospy.Subscriber('/commu', com_msg,callback,pub)
    rospy.spin()

if __name__ == "__main__":
    try:
        rospy.loginfo('starting commu node')
        talker()
    except rospy.ROSInterruptException:
        pass