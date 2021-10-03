#!/usr/bin/env python3

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
from geometry_msgs.msg import Twist

 
idle_state= True




ctl_msg0 = robot_msg(x=0,y=0,isUnload=False)

def callback(ctl_msg,robot):
    twist = Twist()
    r=0.1  #radius
    l=1 #lenght
    #x= (r/2)*(ctl_msg.vl+ctl_msg.vr)
    x=ctl_msg.v
    y=ctl_msg.w
    #x=0.4
    #x = max(min(x, 0.33*0.6), -0.33*0.6)
    msg_robot =robot_msg(x=x,y=y,isUnload=ctl_msg.ifUnload)
    if idle_state:
        msg_robot =robot_msg(x=0,y=0,isUnload=ctl_msg.ifUnload) 
        pass
    twist.linear.x = 0.75
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = ctl_msg.w
    l = x - (y*0.095)/2 
    r = x  + (y*0.095)/2 
     
    l = max(min(l, 0.33), -0.33)
    r = max(min(r, 0.33), -0.33)
    lPwm = l*3000
    rPwm = r*3000
    print("-------->")
    print(lPwm) 
    print(rPwm)
    print("<-----")
    print("~~~~~~")
    print(twist.linear.x )
    print(twist.angular.z)
    #send message to bot
    robot.publish(twist)

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
    robot = rospy.Publisher('/robotMsgs', Twist,queue_size=1)  # May use message format used by bot
    rospy.Subscriber('/commu', com_msg,callback,robot)
    rospy.Subscriber('/state_id',String,stop_navigation)
    rospy.spin()

if __name__ == "__main__":
    try:
        rospy.loginfo('starting commu node')
        talker()
    except rospy.ROSInterruptException:
        pass
