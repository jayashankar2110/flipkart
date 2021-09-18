#!/usr/bin/env python3

"""
CentraolMonitor continuously publish pulse message on heart beat topic so that state-monitor
always check parameter server for start_navigation status. This gives control
to stop and re-start navigation either conditionally or manually.

this node currently monitors feedback node and if feedback is not coming
it will warn and if feedback node is killed then it switches robot to idle state
"""
import rospy
import rosnode
from single_bot.msg import localizemsg
from std_msgs.msg import String
from std_msgs.msg import Bool
import dynamic_reconfigure.client

# nodes = ['/CentralMonitor',
#         '/commu_node',
#         '/localization',
#         '/nav_Client',
#         '/nav_Server',
#         '/rosout',
#         '/state_identifier',
#         '/unload_node']
def callback(msg):
    rospy.loginfo(msg)
    return

def talker():
    rospy.init_node('CentralControl')
    _param_client = dynamic_reconfigure.client.Client("dyn_param_server", timeout=30, config_callback=None)
    beat = rospy.Publisher('/heart_beat', Bool)
    feed_msg = localizemsg()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pulse  =  True
        l = rosnode.get_node_names()
        if '/localzation' in l and '/nav_Client' in l and '/nav_Server' in l :
            pass
        else:
            _param_client.update_configuration({"start_navigation":False})
            #rospy.set_param('/start_navigation', False)
            rospy.logwarn('feedback node is inactive, switching to idle state')
            pulse=False
        beat.publish(pulse)
        rate.sleep()

if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
