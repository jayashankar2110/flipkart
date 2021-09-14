#!/usr/bin/env python3

"""

state_ids are [start, navigate, unload, return, stop]

"""
import rospy
from single_bot.msg import com_msg
from std_msgs.msg import Bool
from std_msgs.msg import String


def unload_cb(msg,c_pub):
    time = 0.0
    unloadTime = rospy.get_param('/unload_time')
    rate = rospy.Rate(2)
    rospy.loginfo(msg)
    if msg.data == 'unload':
        rospy.loginfo('Target reached and unloading started')
        # send signal to commu node for unloading 
        isLoaded = True
        c_pub.publish(v = 0.0,w=0.0,isLoaded=True,ifUnload=True,msg_frm_bot = False)
        #wait for commu node msg
        while isLoaded and not rospy.is_shutdown():
            try:
                isLoaded = rospy.wait_for_message('/unloadStatus', Bool,timeout=unloadTime)
            except:
                ropsy.logwarn('Taking longer than expected to unload..')
                pass
            # time +=0.5
            # if time > unloadTime:
            #     rospy.logwarn('Unload delay detected..')

            if not bot.isLoaded:
                isLoaded = False  #return tray to initial position
                rospy.loginfo('unloaded and returning..')
                
                i_state = rospy.get_param('/i_state')
                f_state = rospy.get_param('/f_state')
                # update job_done at last
                rospy.set_param('/f_state', i_state)
                rospy.set_param('/i_state', f_state)
                rospy.set_param('/c_state', f_state)
                rospy.set_param('/job_done', True)     # indiates return state
            rate.sleep()
    return


def talker():
    rospy.init_node('unload_node', anonymous= False)
    c_pub = rospy.Publisher('/commu', com_msg,queue_size=1)
    rospy.Subscriber('/state_id',String,unload_cb,c_pub)
    rospy.spin()

if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
