#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from single_bot.msg import localizemsg

global time0
time0 = 0.0


def callback(msg):
    global time0
    time = float(rospy.get_time())
    print(time-time0)
    time0=time
    return

def talker():
    global time0
    rospy.init_node('node_tester')
    time0 = float(rospy.get_time())
    #image_pub = rospy.Publisher("image_feedback", Image,queue_size=5)
    pub = rospy.Subscriber('/feedback', localizemsg,callback)
    #rospy.Subscriber("image_topic", Image, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass