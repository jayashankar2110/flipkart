#!/usr/bin/env python3
"""
need to add intermediate point finder
        # define intermediate points
        # xT = waypt[0]
        # yT = waypt[1]
        # phiT = atan2(yT-q[1],xT-q[0])
        # ePhi = phiT - q[2]
        # target waypoint finder
need to add loop for multiple jobs
"""
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import actionlib
import rospy
from CubicSpline import cubic_spline_planner
from pathPlanner import a_star

from single_bot.msg import state1Action
from single_bot.msg import state1Goal
from single_bot.msg import state1Result
from single_bot.msg import state1Feedback
from single_bot.msg import localizemsg
from single_bot.msg import com_msg
import dynamic_reconfigure.client

from math import pi as pi
from math import sin as sin
from math import cos as cos
from math import atan2 as atan2
from math import tan as tan
from math import atan as atan

from PID import PID
import time as clock

import pdb


# Parameters
k = 0.1  # look forward gain
Lfc = 2.0*rospy.get_param('robot_radius')  # [m] look-ahead distance
Kp = 0  # speed proportional gain
dt = rospy.get_param('ctrl_sampling')  # [s] time tick
WB = rospy.get_param('wheel_base')  # [m] wheel base of vehicle


simulation = False
Tune  = True

if simulation:
    show_animation = True
else:
    show_animation = False

yaw_pid = PID.PID(1, 0, 0,clock.time())
yaw_pid.SetPoint=0.0
yaw_pid.setSampleTime(dt)

v_pid = PID.PID(1, 0, 0,clock.time())
v_pid.SetPoint=0.0
v_pid.setSampleTime(dt)

p_pid = PID.PID(1, 0, 0,clock.time())
p_pid.SetPoint=0.0
p_pid.setSampleTime(dt)


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, t =clock.time()):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.t = t
        # self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        # self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta): # for simulatio purpose
        
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        # self.x = self.rear_x #- ((WB / 2) * math.cos(self.yaw))
        # self.y = self.rear_y #- ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)





class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1
            #rospy.loginfo(ind)
        return ind, Lf




class NavigationServer():
    def __init__(self):
        #bot control params
        self.v = 0
        self.w = 0
        self.stopTol=0.05 #in cm
        self.target_vel = 0.5*0.33
        self._max_turning_angle = 0.087 # angle saturatoin
        self.control_mode = None
        cstate=State(x=0, y=0, yaw=0, v=0,t=clock.time()) 
        self.c_state = cstate # we may update velocity via direct feeedback

        self.wpts = [[10,10,10,5,1],[5,3.5,2,2,2]] #updated by path planner
        self.poses = [-pi/2,-pi/2,-pi,-pi,-pi]
        self.wptindx = 0
        #cancel goal if feedback is stuck
        self._feed_time = float(rospy.get_time())
        self._goal_hold_time = 3.0 # in seconds

        #messages
        self.result = state1Result()
        self.action_feedback = state1Feedback()
        self.com_msg = com_msg() 
        
        #nodes
        self._feed_sub = rospy.Subscriber('/feedback',localizemsg,self._feed_cb)
        
        self._com_pub = rospy.Publisher('commu', com_msg,queue_size=1)
        #self._status_sub = rospy.Subscriber('/Monitor', Status, self._status_cb) # to use robot ids 
        self.param_client = dynamic_reconfigure.client.Client("dyn_param_server", timeout=30, config_callback=None)
        
        self._as = actionlib.SimpleActionServer('/Navigation',state1Action,None,auto_start=False)
        self._as.register_goal_callback(self._goal_cb)
        self._as.register_preempt_callback(self._preempt_cb)
        
        #debug messge
        self.debug = {'spin': None, 'alpha':None,'v': None, 'w':None,'control_mode':None}
        
        #result status
        self.success = False
        self.abort = False

        self._as.start()
        rospy.loginfo("Navigation Server started")
    
    # call back functions
    def _feed_cb(self,msg):
        self._feed_time = msg.timestamp
        self.bot_id = msg.id
        if self.bot_id != str(0):  
            self.c_state.x = msg.x_cordinate
            self.c_state.y=msg.y_cordinate
            self.c_state.yaw=msg.angle
            self.c_state.v=msg.velocity
            self.c_state.t=msg.timestamp

            #cstate=State(x=0, y=0, yaw=0, v=0,t=clock.time()) 

        self.send_ctrl()
                        
    def send_ctrl(self):
        #pdb.set_trace()
        #if not self._as.is_active():
        #    self.v = 0
        #    self.w = 0
        vp = self.v
        wp = self.w
        self._com_pub.publish(v = vp, w = wp,ifUnload = False)      
    
    def _preempt_cb(self):
        result = state1Result()
        result.Reached = 'preempted'
        result.tElapsed = self.action_feedback.tElapsed
        rospy.logwarn("prempted")
        self._as.set_preempted(result)
    
    def _success_cb(self):
        result = state1Result()
        result.Reached = 'success' 
        result.tElapsed = self.action_feedback.tElapsed
        self._as.set_succeeded(result)

    def _abort_cb(self):
        result = state1Result()
        result.Reached = 'aborted'
        result.tElapsed = self.action_feedback.tElapsed
        self._as.set_aborted(result)
    
    #utility
    def wrap2PI(self,angle):
        angle = atan(tan(angle))
        return angle
    
    
    def find_next_waypt(self, state, trajectory, pind):
        #pdb.set_trace()
        ind, Lf = trajectory.search_target_index(state)
        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1
        
        #alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw 

        #delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
        return ind


    def plot_arrow(self,x, y, yaw, length=0.01, width=0.05, fc="r", ec="k"):
        """
        Plot arrow
        """

        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                    fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)
    
    def plan_path(self,goal):
        rospy.loginfo("path planning started for given goal")
        a_star.show_animation = False
        # start and goal position
        scaling_factor = rospy.get_param('/mtrs2grid')
        i_state = rospy.get_param('i_state')
        sx = int(i_state[0]*scaling_factor)
        sy = int(i_state[1]*scaling_factor)
        g_state = rospy.get_param('f_state')
        gx = int(g_state[0]*scaling_factor) 
        gy = int(g_state[1]*scaling_factor)
        grid_size = int(rospy.get_param('/grid_size')*scaling_factor) 
        robot_radius = int(rospy.get_param('robot_radius')*scaling_factor) 

        # set obstacle positions currently in grid scale
        ox, oy = [], []
        for i in range(0,100):
            ox.append(i)
            oy.append(0)
        for i in range(0, 50):
            ox.append(100.0)
            oy.append(i)
        for i in range(0,101):
            ox.append(i)
            oy.append(50)
        for i in range(0, 51):
            ox.append(0)
            oy.append(i)
        for i in range(0, 40):
            ox.append(40)
            oy.append(50.0 - i)
        for i in range(0, 40):
            ox.append(60)
            oy.append(50.0 - i)
        for i in range(0,41):
            ox.append(i)
            oy.append(10)
        for i in range(0,41):
            ox.append(100-i)
            oy.append(10)
        
        planner = a_star.AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = planner.planning(gx, gy, sx, sy)
        rx = np.array((rx)).astype(float)/scaling_factor
        ry = np.array((ry)).astype(float)/scaling_factor
        return rx,ry

    #controller mode switcher
    def drive_bot(self):
        #Sampling time
        Ts = dt

        #ref track points
        wpts = np.array(self.wpts)*0.15 #converted to meters bsed
        
        #pdb.set_trace()
        target_course = TargetCourse(wpts[0],wpts[1])
        t_ind, _ = target_course.search_target_index(self.c_state)
        lastIndex = len(wpts[0]) - 1

        pose = self.poses
        qRef = [wpts[0][-1],wpts[1][-1],pose[-1]]
        
        
        
        while not rospy.is_shutdown() and lastIndex >= t_ind: 
            
            rate = rospy.Rate(1/dt)
            D = self.c_state.calc_distance(qRef[0], qRef[1])
            if D < self.stopTol: 
                rospy.loginfo("bot reached goal") 
                self.success = True
                break
            else:
                t_ind = self.find_next_waypt(self.c_state, target_course, t_ind)

                xT = wpts[0][t_ind]
                yT = wpts[1][t_ind]
                phiT = pose[t_ind]

                q = [self.c_state.x,self.c_state.y,self.c_state.yaw]
                
                ePhi = self.wrap2PI(phiT-q[2])
                
                
                if ePhi > pi/2:
                    self.v = 0.1    #orientation contrl
                    self.head_ctl(ePhi)
                if t_ind == lastIndex:
                    self.pose_ctl(D)
                    self.head_ctl(ePhi)
                else:
                    ePhi = max(min(ePhi, self._max_turning_angle), -self._max_turning_angle  )
                    e_vel = self.target_vel - self.c_state.v
                    self.vel_ctl(e_vel)
                    self.head_ctl(ePhi)
            rate.sleep()
        # if lastIndex < t_ind:
        #     self.abort = True
    
    def head_ctl(self,ePhi):
        if Tune:
            k = self.param_client.get_configuration(timeout=1)
            kp= float(k['yaw_p'])
            kr= float(k['yaw_rp'])   #will be passed to double_update          
            yaw_pid.setKp(kp)

        yaw_pid.double_update(ePhi,kr,clock.time())
        wd = yaw_pid.output
        self.w = self.w + wd
    
    def vel_ctl(self,e_vel):
        if Tune:
            k = self.param_client.get_configuration(timeout=1) 
            kp= float(k['vel_p'])
            kd= float(k['vel_d']) 
            ki= float(k['vel_i'])
            v_pid.setKp(kp)
            v_pid.setKd(kd)
            v_pid.setKi(ki)
        
        v_pid.update(e_vel,clock.time())
        a = v_pid.output
        self.v += a*dt
    
    def pos_ctl(self,e_pos):
        if Tune:
            k = self.param_client.get_configuration(timeout=1) 
            kp= float(k['pos_p'])
            kd= float(k['pos_d']) 
            ki= float(k['pos_i'])
            p_pid.setKp(kp)
            p_pid.setKd(kd)
            p_pid.setKi(ki)
        
        p_pid.update(e_pos,clock.time())
        self.v = p_pid.output

    def _goal_cb(self):

        if self._as.is_active() or not self._as.is_new_goal_available():
            return
        
        rospy.loginfo('Received a new request to start navigation')# of bot: %s' % goal.behavior_name)
        goal = self._as.accept_new_goal()
        wpts = rospy.get_param('way_points')
        ax = wpts[0]
        ay = wpts[1]

        #identify direction of travel
        i_state = rospy.get_param('i_state')
        if i_state=='f':
            ax.reverse()
            ay.reverse()
        #find intermediate points
        ds = 0.5*rospy.get_param('robot_radius') # way points distance
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds)

        #update waypoints
        self.wpts = [cx,cy]
        self.poses = cyaw

        #navigate
        while not rospy.is_shutdown():
            self.drive_bot()
            
            if self.success:
                self._success_cb()
            if self.abort:
                self._abort_cb() 
            

if __name__ == '__main__':
    rospy.init_node('nav_Server')
    server = NavigationServer()
    rospy.spin()

