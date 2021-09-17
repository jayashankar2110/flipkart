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

"""
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import actionlib
import rospy
from CubicSpline import cubic_spline_planner
from pathPlanner import a_star
from rospy.core import loginfo

from single_bot.msg import state1Action
from single_bot.msg import state1Goal
from single_bot.msg import state1Result
from single_bot.msg import state1Feedback
from single_bot.msg import localizemsg
from single_bot.msg import com_msg
import dynamic_reconfigure.client


from PID import PID
import time as clock
import pdb

# Parameters
k = 0.1  # look forward gain
Lfc = 1.0*rospy.get_param('/robot_radius')  # [m] look-ahead distance
Kp = 0.5  # speed proportional gain
global dt
dt = rospy.get_param('/ctrl_sampling')  # [s] time tick
WB = rospy.get_param('/wheel_base')  # [m] wheel base of vehicle


simulation = False

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
                ##rospy.logwarn("searching...")
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
            ##rospy.logwarn("look ahead")
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1
            #rospy.loginfo(ind)
        return ind, Lf



class NavigationServer():
    def __init__(self):
        self.v = 0
        self.w = 0
        self.v_output=0
        self.yaw_output=0
        
        self.c_state = None
        self.bot_id = 0
        self.bot_L = None
        self._cancelRequest = False
        self._feed_time = float(rospy.get_time())
        self._goal_hold_time = 3.0 # in seconds
        self._max_turning_angle = math.pi/3  # if change in angle is beyond this robot velocity will be not be updated
        self.action_feedback = state1Feedback()
        self.com_msg = com_msg() 
        self.current_time = clock.time()
        self.last_time = clock.time()
        self.last_error = 0

        #self._active_behavior_id = None
        self._feed_sub = rospy.Subscriber('/feedback',localizemsg,self._feed_cb)
        self._com_pub = rospy.Publisher('/commu', com_msg,queue_size=1)
        #self._status_sub = rospy.Subscriber('/Monitor', Status, self._status_cb) # to use robot ids 
        self._as = actionlib.ActionServer('/Navigation',state1Action,goal_cb=self._goal_cb,cancel_cb = self._cancel_cb, auto_start=False)
        self.param_client = dynamic_reconfigure.client.Client("dyn_param_server", timeout=30, config_callback=None)
        self._as.start()
        rospy.loginfo("Navigation Server started")
    

    # call back functions
    def _feed_cb(self,msg):
        self._feed_time = msg.timestamp
        self.bot_id = msg.id
        if self.bot_id != str(0):
            
            self.c_state = [msg.x_cordinate,msg.y_cordinate,msg.angle,msg.velocity] 
            #yaw = self.wrap2PI(float(msg.angle))


    
    def _cancel_cb(self,goal_handle):
        ##rospy.logwarn('cancel request received')
        self._cancelRequest = True
    
    def _preempt_cb(self,goal_handle):
        result = state1Result()
        rospy.loginfo("preempted")
        result.Reached = 'preempted'
        result.tElapsed = self.action_feedback.tElapsed
        goal_handle.set_canceled(result)
    
    def _success_cb(self,goal_handle):
        result = state1Result()
        rospy.loginfo("success")
        result.Reached = 'success' 
        result.tElapsed = self.action_feedback.tElapsed
        goal_handle.set_succeeded(result)

    def _abort_cb(self,goal_handle):
        result = state1Result()
        rospy.loginfo("aborted as feedback not updating")
        result.Reached = 'aborted'
        result.tElapsed = self.action_feedback.tElapsed
        goal_handle.set_aborted(result)
    
    def vel_control(self,target, current,Kp):
        windup_guard = 20.0
        error = math.sqrt( ((target[0]-current[0])**2)+((target[1]-current[1])**2) )
        self.current_time = clock.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= dt):
            PTerm = Kp * error
            # ITerm=0
            # ITerm += error * delta_time

            # if ( ITerm < windup_guard):
            #     ITerm = -windup_guard
            # elif (ITerm > windup_guard):
            #     ITerm=windup_guard
            # DTerm = 0.0
            # if delta_time > 0:
            #     DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.v_output = PTerm
            return self.v_output
    
    def wrap2PI(self,angle):
        angle = (angle + np.pi) % (2 * np.pi) - np.pi

        return angle
    
    def yaw_control(self,SetPoint,feedback_value,Kp,Ki,Kd):
        windup_guard = 20.0
        error = SetPoint - feedback_value
        if error < 0.05 and error > -0.05:
            error = 0
        rospy.loginfo(error)
        self.current_time = clock.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= dt):
            PTerm = Kp * error
            ITerm=0
            ITerm += error * delta_time

            if ( ITerm < windup_guard):
                ITerm = -windup_guard
            elif (ITerm > windup_guard):
                ITerm=windup_guard
            DTerm = 0.0
            if delta_time > 0:
                DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.yaw_output = PTerm + (Ki * ITerm) + (Kd * DTerm)
            return self.yaw_output
    
    def send_ctrl(self, a, delta):
        # if delta >= math.pi/3:
        #     a =0
        self.w = self.wrap2PI(delta)   #self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        # bot_vr = self.v + (self.bot_L*self.w)/2
        # bot_vl = self.v - (self.bot_L*self.w)/2
        self._com_pub.publish(v = self.v, w = self.w,ifUnload = False)
        #self._com_pub.publish(vr = bot_vr, vl = bot_vl,ifUnload = False)
    
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
        #print(tx,ty)
        
        # alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw 

        # delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
        #pdb.set_trace()
        theta = state.yaw
        #theta_r = math.atan2(ty - state.y, tx - state.x)
        T = [[math.cos(theta),math.sin(theta)],[-math.sin(theta),math.cos(theta)]]
        
        #ref_pos = np.dot(T,[[tx],[ty]])
        #ref_pos = np.dot(T,[[tx],[ty]]) - [[state.x],[state.y]]
        ref_pos = np.dot(T,[[tx-state.x],[ty-state.y]])
        alpha = math.atan2(ref_pos[1], ref_pos[0])
        #rospy.loginfo(alpha)
        if alpha > math.pi/3:
            alpha = math.pi/3
        
        #k = self.param_client.get_configuration(timeout=1)
        #yaw_pid.setKp = float(k['yaw_p']) 
        #yaw_pid.setKi = float(k['yaw_i']) 
        #yaw_pid.setKd= float(k['yaw_d']) 
        k = self.param_client.get_configuration(timeout=1)
        kp = float(k['yaw_p']) 
        ki = float(k['yaw_i']) 
        kd = float(k['yaw_d']) 
        kv= float(k['vel_p']) 

        print(kv)
        delta = self.yaw_control(0,alpha,kp,ki,kd)
        target_pose = [tx,ty]
        curr_pose = [state.x,state.y]
        #pdb.set_trace()
        #ai = self.vel_control(target_pose,curr_pose,kv)
        ai=0
        #pdb.set_trace()
        #yaw_pid.update(alpha,clock.time())
        #delta = yaw_pid.output 
        return delta,ai, ind


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
        i_state = rospy.get_param('/i_state')
        sx = int(i_state[0]*scaling_factor)
        sy = int(i_state[1]*scaling_factor)
        g_state = rospy.get_param('/f_state')
        gx = int(g_state[0]*scaling_factor) 
        gy = int(g_state[1]*scaling_factor)
        grid_size = int(rospy.get_param('/grid_size')*scaling_factor) 
        robot_radius = int(rospy.get_param('/robot_radius')*scaling_factor) 
        
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

    def process_goal(self,goal_handle):
        goal = goal_handle.get_goal()
        rate = rospy.Rate(1/dt)
        #ax,ay = self.plan_path(goal)
        scaling_factor = rospy.get_param('/mtrs2grid')
        #ay=np.array([7,7,7,7,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2])
        #ax=np.array([10,10,10,10,10,10,10,10,10,5,5,5,5,5,1,1,1,1,1])
        ax=np.array([10,10,5,1])
        ay=np.array([7,2,2,2])
        cx=(ax-1)*0.15
        cy=(ay-1)*0.15
        #rospy.loginfo(str(ax))
        #rospy.loginfo(str(ay))
        #pdb.set_trace()
        ds = 1.5*rospy.get_param('/robot_radius') # way points distance
        #cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds)

        wheelR = rospy.get_param('/wheel_radius')
        motor_RPM = rospy.get_param('/motor_max_speed')  # in RPM
        target_speed =  motor_RPM*0.10472*wheelR  # [m/s]
        self.bot_L = rospy.get_param('/wheel_base')

        T = rospy.get_param('/duration')  # max simulation time
        
        lastIndex = len(cx) - 1
        # update initial state
        if simulation:
            self.c_state = rospy.get_param('/c_state')
        else:
            if self.c_state is None:
                self.c_state = rospy.get_param('/c_state')
        
        state = State(x=self.c_state[0], y=self.c_state[1], yaw=self.c_state[2], v=self.c_state[3]) # we may update velocity via direct feeedback
        
        # initialise parameters
        self.action_feedback.final_wpt_indx = lastIndex
        time = 0.0
        
        if simulation:
            states = States()
            states.append(time, state)
        
        target_course = TargetCourse(cx, cy)
        target_ind, _ = target_course.search_target_index(state)
        
        preempted = False
        abort = False
        #while T >= time and lastIndex > target_ind and not rospy.is_shutdown():
        while not rospy.is_shutdown():

            # check if bot is under camera
            while self.bot_id ==str(0)and not self._cancelRequest:
                rospy.logwarn('No bot detected, navigation halted..')
                rospy.sleep(1.0)
            
            #check if client requested to cancel goal
            if self._cancelRequest:
                self._cancelRequest = False
                preempted = True
                break
            #check if feedback is updated
            t = float(rospy.get_time())
            if t - self._feed_time > self._goal_hold_time:
                rospy.logwarn("feedback not updated")
                abort = True
                break
            
            #start navigation
            if not simulation:
                state = State(x=self.c_state[0], y=self.c_state[1], yaw=self.c_state[2], v=self.c_state[3]) # we may update velocity via direct feeedback
                #rospy.loginfo(state.yaw)
            # Calc control input
            #rospy.logwarn("123")
            
            di,ai,target_ind = self.pure_pursuit_steer_control(state, target_course, target_ind)
            #rospy.logwarn("456")
            #update way_point index in feedback msg
            self.action_feedback.target_wpt_indx = target_ind
            if simulation:
                state.update(ai, di)  #update inital states
            self.send_ctrl(ai,di) #send control input to commu node
            ##rospy.logwarn("send control")

            # update time elapsed
            time += dt 
            self.action_feedback.tElapsed = time
            
            if simulation:
                states.append(time, state)
            
            #update deviation form path and next pose
            self.action_feedback.nextPose = [cx[target_ind], cy[target_ind]]
            self.action_feedback.deviation = state.calc_distance(cx[target_ind], cy[target_ind])
            #publish feedback using aciton server
            goal_handle.publish_feedback(self.action_feedback)

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
            
            #tuning param loop delay comment once it is tuned
            k = self.param_client.get_configuration(timeout=1)
            loop_delay = float(k['loop_delay']) 
            rospy.sleep(loop_delay)    
        if preempted:
            self._preempt_cb(goal_handle)
        if abort:
            self._abort_cb(goal_handle)

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
            self._success_cb(goal_handle)   
        # else:
        rospy.loginfo("while end")
    
    def _goal_cb(self,goal_handle):
        result = []
        result = state1Result()
        self.action_feedback = []
        self.action_feedback = state1Feedback()

        # validate and receive goal request
        rospy.loginfo('Received a new request to start navigation')# of bot: %s' % goal.behavior_name)
        
        goal_handle.set_accepted()
        self.process_goal(goal_handle)                 #self._abort_cb(result)
        return    

if __name__ == '__main__':
    rospy.init_node('nav_Server')
    server = NavigationServer()
    rospy.spin()
