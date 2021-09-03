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
from single_bot.msg import robot_msg
from single_bot.msg import com_msg
from std_msgs.msg import String

 
idle_state= True




ctl_msg0 = robot_msg(x=0,y=0,isUnload=False)

def callback(ctl_msg,robot):
    r=0.1  #radius
    l=1 #lenght
    x= (r/2)*(ctl_msg.vl+ctl_msg.vr)
    y=(r/l)*(ctl_msg.vl-ctl_msg.vr)
    msg_robot =robot_msg(x=x,y=y,isUnload=ctl_msg.isUnload)
    if idle_state:
        msg_robot =robot_msg(x=0,y=0,isUnload=ctl_msg.isUnload) 
        pass
    
    #send message to bot
    robot.publish(msg_robot)
    # save previous control input for future developments
    ctl_msg0 = ctl_msg
    return

def stop_navigation(msg):
    global idle_state
    if msg.data == 'Idle': 
        idle_state = True
        rospy.loginfo('bot in idle_state')
    else:
        idle_state = False
    return

def talker():
    rospy.init_node('commu_node')
    robot = rospy.Publisher('/robotMsgs', robot_msg,queue_size=1)  # May use message format used by bot
    rospy.Subscriber('/commu', com_msg,callback,robot)
    rospy.Subscriber('/state_id',String,stop_navigation)
    rospy.spin()

if __name__ == "__main__":
    try:
        rospy.loginfo('starting commu node')
        talker()
    except rospy.ROSInterruptException:
        pass