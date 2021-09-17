#!/usr/bin/env python

import rospy
from single_bot.msg import localizemsg




def talker():

    rospy.init_node('localzation')
    
    pub = rospy.Publisher('/feedback', localizemsg, queue_size=10)

    while not rospy.is_shutdown():
        feed = localizemsg(angle= 0.0, x_cordinate= 10.0, y_cordinate= 7.0, velocity=0.0,timestamp = float(rospy.get_time()),id= '1')
        pub.publish(feed)
        rospy.sleep(0.5)

if __name__ == "__main__":
    try:
        rospy.loginfo('dummy feed started')
        talker()
    except rospy.ROSInterruptException:
        pass