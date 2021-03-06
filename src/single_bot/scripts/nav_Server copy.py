#!/usr/bin/env python3
"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
########
modified as ros action. with action messages  as follows
goal msgs - inital and final positions for path planning and tracking float64[]
          - maximum time duration for reaching goal float[64] which publish timout warning to monitor
result msgs - bool success, float64 tElapsed 
feedback msgs - float64 tElapsed, float64 deviation, float64[] nextPose, string message

#####ISSUES###
TODO:  Need to give abort signal when feedback stops
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

from PID import PID
import time as clock

import pdb
# feedback msg
dt  =rospy.get_param('ctrl_sampling')
#global c_state

# Parameters
k = 0.1  # look forward gain
Lfc = 1.0*rospy.get_param('robot_radius')  # [m] look-ahead distance
Kp = 0.5  # speed proportional gain
dt = rospy.get_param('ctrl_sampling')  # [s] time tick
WB = rospy.get_param('wheel_base')  # [m] wheel base of vehicle


simulation = True

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

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        # self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        # self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        if delta >= math.pi/3:
            v =0
            a =0
        
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
        self.v = 0
        self.w = 0
        self.action_feedback = state1Feedback()
        self.com_msg = com_msg() 
        #self._active_behavior_id = None
        self._feed_sub = rospy.Subscriber('/feedback',localizemsg,self._feed_cb)
        self._com_pub = rospy.Publisher('/commu', com_msg,queue_size=1)
        #self._status_sub = rospy.Subscriber('/Monitor', Status, self._status_cb) # to use robot ids 
        self._as = actionlib.SimpleActionServer('/Navigation',state1Action,None,auto_start=False)
        self._as.register_goal_callback(self._goal_cb)
        self._as.register_preempt_callback(self._preempt_cb)
        self._as.start()
        rospy.loginfo("Navigation Server started")
    
    # call back functions
    def _feed_cb(self,msg):
        if self._as.is_active() or self._as.is_new_goal_available():
            global c_state
            c_state = [msg.x_cordinate,msg.y_cordinate,msg.angle,msg.velocity]  #robot current state to update using subscriber cb
        #rospy.loginfo(c_state)

    
    def _preempt_cb(self):
        result = state1Result()
        rospy.loginfo("preempted")
        result.Reached = 'preempted'
        result.tElapsed = self.action_feedback.tElapsed
        self._as.set_preempted(result)
    
    def _success_cb(self,result):
        rospy.loginfo("success")
        result.Reached = 'success' 
        result.tElapsed = self.action_feedback.tElapsed
        self._as.set_succeeded(result)

    def _abort_cb(self,result):
        rospy.loginfo("aborted")
        result.Reached = 'aborted'
        result.tElapsed = self.action_feedback.tElapsed
        self._as.set_aborted(result)
    
    def proportional_control(self,target, current):
        v_pid.SetPoint=target
        v_pid.update(current,clock.time())
        a = v_pid.output
        #a = Kp * (target - current)
        return a
    
    def send_ctrl(self, a, delta):
        if delta >= math.pi/3:
            a =0
        self.w = self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self._com_pub.publish(v = self.v, w = self.w,ifUnload = False)
    
    def pure_pursuit_steer_control(self, state, trajectory, pind):
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
        
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw 

        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
        return delta, ind


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
        scaling_factor = rospy.get_param('mtrs2grid')
        i_state = rospy.get_param('i_state')
        sx = int(i_state[0]*scaling_factor)
        sy = int(i_state[1]*scaling_factor)
        g_state = rospy.get_param('f_state')
        gx = int(g_state[0]*scaling_factor) 
        gy = int(g_state[1]*scaling_factor)
        grid_size = int(rospy.get_param('grid_size')*scaling_factor) 
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

    
    def _goal_cb(self):
        result = []
        result = state1Result()
        self.action_feedback = []
        self.action_feedback = state1Feedback()

        if self._as.is_active() or not self._as.is_new_goal_available():
            return
        
        # accept goal and send it to navigate function, get action feedback message
        rospy.loginfo('Received a new request to start navigation')# of bot: %s' % goal.behavior_name)
        goal = self._as.accept_new_goal()

        nh = self._as

        rate = rospy.Rate(1/dt)
        ax,ay = self.plan_path(goal)
        ds = 1.5*rospy.get_param('robot_radius') # way points distance
        #pdb.set_trace()
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds)
        wheelR = rospy.get_param('wheel_radius')
        motor_RPM = rospy.get_param('motor_max_speed')  # in RPM
        target_speed =  motor_RPM*0.10472*wheelR  # [m/s]

        T = rospy.get_param('duration')  # max simulation time
        
        lastIndex = len(cx) - 1
        # update initial state
        global c_state
        if simulation:
            c_state = rospy.get_param('c_state')
        else:
            if c_state is None:
                c_state = rospy.get_param('c_state')
        
        state = State(x=c_state[0], y=c_state[1], yaw=c_state[2], v=0.0) # we may update velocity via direct feeedback
        
        # initialise parameters
        self.action_feedback.final_wpt_indx = lastIndex
        time = 0.0
        if simulation:
            states = States()
            states.append(time, state)
        target_course = TargetCourse(cx, cy)
        target_ind, _ = target_course.search_target_index(state)
        

        while T >= time and lastIndex > target_ind and not rospy.is_shutdown():
            #uncomment for real feedback to update state in loop
            if not simulation:
                state = State(x=c_state[0], y=c_state[1], yaw=c_state[2], v=0.0)
            
            # Calc control input
            ai = self.proportional_control(target_speed, state.v)
            di, target_ind = self.pure_pursuit_steer_control(state, target_course, target_ind)
            
            #update way_point index in feedback msg
            self.action_feedback.target_wpt_indx = target_ind
            
            state.update(ai, di)  #update inital states
            self.send_ctrl(ai,di) #send control input to commu node

            # update time elapsed
            time += dt 
            self.action_feedback.tElapsed = time
            
            if simulation:
                states.append(time, state)
            
            #update deviation form path and next pose
            self.action_feedback.nextPose = [cx[target_ind], cy[target_ind]]
            self.action_feedback.deviation = state.calc_distance(cx[target_ind], cy[target_ind])
            #publish feedback using aciton server
            nh.publish_feedback(self.action_feedback)

            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                self.plot_arrow(state.x, state.y, state.yaw)
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(states.x, states.y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)
            
            # feedback
        

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            plt.subplots(1)
            plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)
            plt.show()

        if  self.action_feedback.final_wpt_indx <= self.action_feedback.target_wpt_indx:
            self._success_cb(result)   
        # else:
        #         #self._abort_cb(result)
            

if __name__ == '__main__':
    rospy.init_node('nav_Server')
    server = NavigationServer()
    rospy.spin()

