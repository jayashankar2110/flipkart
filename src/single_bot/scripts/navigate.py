#!/usr/bin/env python3
import rospy


import dynamic_reconfigure.client
from PID import PID
import time as clock
import pdb
import math
from single_bot.msg import localizemsg
from single_bot.msg import com_msg

import numpy as np

# paramers
look_forward = 0.20  # in cm
param_client = dynamic_reconfigure.client.Client("dyn_param_server", timeout=30, config_callback=None)
  

class Bot():
    def __init__(self,id):
        self.bot_id = id
        self.cx=[]
        self.cy=[] 
        self.v = 0
        self.w=0
        self.c_state=None
        self.param_client = dynamic_reconfigure.client.Client("dyn_param_server", timeout=30, config_callback=None)
        self.debug = dict()
        self.debug = {}
        self.target_indx = 0

    def control(self,_com_pub):
        k = self.param_client.get_configuration(timeout=1) 
        drive = k['start_navigation']
        while drive and self.target_indx < len(self.cx) and not rospy.is_shutdown():
            #GUI Control
            drive = k['start_navigation']
            # determine target point
            dist = self.calc_distance(self.cx[self.target_indx], self.cy[self.target_indx])
            if dist < look_forward:
                self.target_indx += 1
            tx = self.cx[self.target_indx]
            ty = self.cy[self.target_indx]
            #get feedback and transform error
            theta = self.c_state[2]
            x = self.c_state[0]
            y = self.c_state[1]

            T = [[math.cos(theta),math.sin(theta)],[-math.sin(theta),math.cos(theta)]]
            Xe = np.dot(T,[[tx-x],[ty-y]]) # error vector
            eTheta = math.atan2(Xe[1], Xe[0])
            eTheta = -max(min(eTheta, math.pi/1.5), -math.pi/1.5)
            ePos = np.sqrt((tx-x)^2 + (ty-y)^2)

            #calculate control input
            kv= float(k['vel_p'])
            kp= float(k['yaw_p'])
            v = kv*ePos
            w = math.tan(math.atan(kp*eTheta))
            self.v = v
            self.w = w
            _com_pub.publish(v = self.v, w = self.w,ifUnload = False)
            # break if unstable
            if ePos > 40:
                rospy.logwarn("bot is too far from target")
                break

            # debug output
            self.debug['Xe'] = ePos
            self.debug['alpha_e'] = eTheta
            self.debug['control'] = [v,w]
            self.debug['target_indx'] = self.target_indx
            print(self.debug)
        print('Robot stopped')
        self.v= 0
        self.w= 0
        param_client.update_configuration({"start_navigation":False})
        

    def calc_distance(self, point_x, point_y):
        #pdb.set_trace()
        x = self.c_state[0]
        y = self.c_state[1]
        dx = x - point_x
        dy = y - point_y
        return math.hypot(dx, dy)

def feed_cb(msg,bot):
    #if bot.bot_id in msg.id:
    bot.c_state = [msg.x_cordinate,msg.y_cordinate,msg.angle,msg.velocity] 
        #yaw = self.wrap2PI(float(msg.angle))


if __name__ == '__main__':
    rospy.init_node('nav_Server')
    ax=np.array([10,10,10,5,1])
    ay=np.array([5,3.5,2,2,2]) 
    bot1 = Bot(str(1))   # bot id
    bot1.cx = (ax-1)*0.15
    bot1.cy = (ay-1)*0.15
    bot1.target_indx = 0
    _pub_cntl = rospy.Publisher('/commu', com_msg,queue_size=1)
    feed_sub = rospy.Subscriber('/feedback',localizemsg,feed_cb,bot1)
    bot1.control(_pub_cntl)
    rospy.spin()