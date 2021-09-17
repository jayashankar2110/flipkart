#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image


time0 = 0.0


def callback(msg):
    
    time = float(rospy.get_time())
    print(time-time0)
    return

def talker():
   
    rospy.init_node('node_tester')
    time0 = float(rospy.get_time())
    #image_pub = rospy.Publisher("image_feedback", Image,queue_size=5)
    #pub = rospy.Publisher('/feedback', localizemsg, queue_size=10)
    rospy.Subscriber("image_topic", Image, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass