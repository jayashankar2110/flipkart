#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {start_navigation}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("dyn_param_server", timeout=30, config_callback=None)

    r = rospy.Rate(0.1)
    x = 0
    b = False
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0
        b = not b
        k = client.get_configuration(timeout=1)
        rospy.loginfo(k["start_navigation"])
        client.update_configuration({"start_navigation":b})
        r.sleep()