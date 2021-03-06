#!/usr/bin/env python3


import rospy

from dynamic_reconfigure.server import Server
from single_bot.cfg import navigationConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {start_navigation},{loop_delay},{yaw_p},{yaw_i},{yaw_d},{vel_p}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dyn_param_server", anonymous = False)
    srv = Server(navigationConfig,callback)
    rospy.spin()