#!/usr/bin/env python

"""
This node works at rate of heart beat and starts pushing robot states as per parameter values
Heart beat is just to keep in sync with centralcontrol, we dont use that message
"""



from std_msgs.msg import String
from std_msgs.msg import Bool
import rospy

global state_id0
state_id0 = 'init'

def publish_state_id(pulse,pub):

    #rospy.loginfo('checking state')
    global state_id0
    state_id = None
    c_state = rospy.get_param('c_state')
    f_state = rospy.get_param('f_state')
    job_done = rospy.get_param('job_done')
    start_navigation = rospy.get_param('start_navigation')

    # if c_state == i_state and not job_done:
    if start_navigation and pulse:
        if c_state!= f_state and not job_done:
            rospy.loginfo('sending bot to unload')
            state_id = 'navigate'
        if c_state == f_state and not job_done:
            rospy.loginfo('unloading')
            state_id = 'unload'
        if c_state != f_state and job_done:
            rospy.loginfo('Retuning to start position')
            state_id = 'navigate'
    elif pulse == True:
        state_id = 'hold'
        #rospy.loginfo('waiting for camera feedback..')
    else:
        state_id = 'Idle'
        #rospy.loginfo('Entered Idle  State')
    
    if state_id and state_id0 != state_id:
        pub.publish(state_id)
        rospy.loginfo('state id set to ' + str(state_id))
        state_id0 = state_id
    if not state_id:
        rospy.logwarn('stateid_not identified..')
        #pdb.set_trace()
    return


def talker():
    rospy.init_node('state_identifier')
    state_pub = rospy.Publisher('/state_id',String, queue_size=1 )
    rospy.Subscriber('/heart_beat',Bool,publish_state_id,state_pub)
    rospy.spin()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInitException():
        rospy.loginfo('State Identifier Terminated')