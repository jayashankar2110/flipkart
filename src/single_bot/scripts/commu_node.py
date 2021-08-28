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
from std_msgs.msg import String

global idle_state 
idle_state= True

global com_msg0


ctl_msg0 = com_msg(vr=0,vl=0,ifUnload=False)

def callback(ctl_msg,robot):
    #write code to calculate twist message or vr,vl of robot 
    #----
    #----

    # stop bot if bot state is switched to idle_state
    if idle_state:
        # modify message to stop bot
        pass
    
    #send message to bot
    robot.publish(ctl_msg)
    # save previous control input for future developments
    ctl_msg0 = ctl_msg
    return

def stop_navigation(msg):
    global idle_state
    if msg == 'Idle': 
        idle_state = True
        rospy.loginfo('bot in idle_state')
    else:
        idle_state = False
    return

def talker():
    rospy.init_node('commu_node')
    robot = rospy.Publisher('/robotMsgs', com_msg,queue_size=1)  # May use message format used by bot
    rospy.Subscriber('/commu', com_msg,callback,robot)
    rospy.Subscriber('/state_id',String,stop_navigation)
    rospy.spin()

if __name__ == "__main__":
    try:
        rospy.loginfo('starting commu node')
        talker()
    except rospy.ROSInterruptException:
        pass