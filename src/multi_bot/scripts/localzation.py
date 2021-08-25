#!/usr/bin/env python

"""
node name 'listner', feed_publishes bot state to '/feedback' topic in message format as

angle: 8.86922925173
x_cordinate: 10
y_cordinate: 0
timestamp: 0.0
id: "90145110"

"""
import rospy
from flipkart.msg import localizemsg
import cv2 as cv
import math
import numpy as np
from os.path import sep
from std_msgs.msg import String

#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
state_id0 = 'init'
start=0
old_gray=[[]]
color=[]
mask=[[]]
p0 = np.array([[532*0.6,369*0.6],[530*0.6,480*0.6]], dtype="float32").reshape(-1,1,2) #pomits to track
msg=localizemsg()
feed_pub=None
lk_params = dict( winSize  = (15,15),
                        maxLevel = 2,
                        criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))



class good_oldpts():
    def __init__(self):
        self.good_new= []
        self.good_old=[]

def find_angle(a,b,center_a,center_b):
    angle=math.atan2((center_a-a),(center_b-b))
    if angle<0:
        angle=angle+2*math.pi

    angle=math.degrees(angle)
    return angle
def localize(frame,goodpts):
    global old_gray
    global p0
    global lk_params
    global color
    global mask
    global msg
    global feed_pub
    pts1 = np.float32([[62, 0], [630, 0], [62, 320], [625, 320]])
    pts2 = np.float32([[0, 0], [630,0], [0,320], [630, 320]])
      
    # Apply Perspective Transform Algorithm
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    result = cv.warpPerspective(frame, matrix, (600, 600))
    res1 = cv.warpPerspective(frame, matrix, (600, 280))
    frame_gray = cv.cvtColor(res1, cv.COLOR_BGR2GRAY)
    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # Select the points
    if p1 is not None:
        goodpts.good_new = p1[st==1]
        goodpts.good_old = p0[st==1]
    good_new = goodpts.good_new
    good_old = goodpts.good_old

    # draw the tracks
    center_a=0
    center_b=0
    for i,(new,old) in enumerate(zip(good_new, good_old)):
        rospy.sleep(0.01)
        a,b = new.ravel()
        c,d = old.ravel()
        #mask = cv.line(mask, (int(a),int(b)),(int(c),int(d)), color[i//2].tolist(), 2)
        #res1 = cv.circle(res1,(int(a),int(b)),5,color[i].tolist(),-1)
        if i%2==0:
            center_a=a
            center_b=b
            res1 = cv.circle(res1,(int(a),int(b)),5,color[i//2].tolist(),-1)
            mask = cv.line(mask, (int(a),int(b)),(int(c),int(d)), color[i//2].tolist(), 2)
        else:
            angle=find_angle(a,b,center_a,center_b)
            res1 = cv.arrowedLine(res1, (int(center_a),int(center_b)),(int(a),int(b)), color[i//2].tolist(), 1)
            ids=color[i//2].tolist()
            iden=str(ids[0])+str(ids[1])+str(ids[2])
            print('x: {} \n y: {} angle: {} id: {}'.format(center_a//31.5,center_b//27,angle,iden))
            msg = localizemsg()
            msg.x_cordinate = center_a//31.5
            msg.y_cordinate = center_b//27
            msg.angle = angle
            msg.id = iden
            msg.timestamp = 0.0
            feed_pub.publish(msg)
            publish_state_id(msg, state_pub)
            
    img = cv.add(res1,mask)
    cv.imshow('frame',img)
    
    k = cv.waitKey(1) & 0xff
    #if k == 27:
    #    break
    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1,1,2)

def start_up(data):
    global start
    global old_gray
    global p0
    global color
    global mask
    global goodpts
    if start==0:
       #"bridge" bridge = CvBridge()
        # params for ShiTomasi corner detection

        # Parameters for lucas kanade optical flow
        
        # Create some random colors
        color = np.random.randint(0,255,(100,3))
        # Take first frame and find corners in it

        old_frame = data
        #old_frame = cv.resize()
        #pts1 = np.float32([[62, 0], [630, 0], [62, 320], [625, 320]])
        #pts2 = np.float32([[0, 0], [630,0], [0,320], [630, 320]])
            
            # Apply Perspective Transform Algorithm
        #matrix = cv.getPerspectiveTransform(pts1, pts2)
        #result = cv.warpPerspective(old_frame, matrix, (600, 600))
        #res = cv.warpPerspective(old_frame, matrix, (600, 280))
        old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
        #p0 = np.array([[315,280],[315,265]], dtype="float32").reshape(-1,1,2) #pomits to track
        #print(p0)
        # Create a mask image for drawing purposes
        mask = np.zeros_like(old_frame)
        start=1
        #update c_state in parameter space
    else:
        
        localize(data,goodpts)
    

import pdb
def publish_state_id(msg,pub):
    #uncomment if realtime
    #c_state = [msg.x_cordinate, msg.y_cordinate, msg.angle]
    # state_id params
    global state_id0
    state_id = None
    c_state = rospy.get_param('c_state')
    f_state = rospy.get_param('f_state')
    job_done = rospy.get_param('job_done')

    # if c_state == i_state and not job_done:
    if c_state!= f_state and not job_done:
        rospy.loginfo('sending bot to unload')
        state_id = 'navigate'
    if c_state == f_state and not job_done:
        rospy.loginfo('unloading')
        state_id = 'unload'
    if c_state != f_state and job_done:
        rospy.loginfo('Retuning to start position')
        state_id = 'navigate'
    # if c_state == f_state and job_done:
    #     rospy.loginfo('Job done')
    #     state_id = 'stop'
    if state_id and state_id0 != state_id:
        pub.publish(state_id)
        #rospy.loginfo(state0,state)
        state_id0 = state_id
    else:
        rospy.logwarn('stateid_not identified..')
        #pdb.set_trace()
    return

    
def talker():
    global feed_pub
    global state_pub
    global start
    global old_gray
    global p0
    global color
    global mask
    global goodpts
    rospy.init_node('listener')
    feed_pub = rospy.Publisher('/feedback', localizemsg, queue_size=1)
    state_pub = rospy.Publisher('/state_id',String, queue_size=1 )
    cap = cv.VideoCapture('src/flipkart/scripts/test8.mp4')
    # params for ShiTomasi corner detection

    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15,15),
                    maxLevel = 2,
                    criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    # Create some random colors
    color = np.random.randint(0,255,(100,3))
    # Take first frame and find corners in it

    ret, old_frame = cap.read()
    pts1 = np.float32([[62, 0], [630, 0], [62, 320], [625, 320]])
    pts2 = np.float32([[0, 0], [630,0], [0,320], [630, 320]])
        
    # Apply Perspective Transform Algorithm
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    result = cv.warpPerspective(old_frame, matrix, (600, 600))
    res = cv.warpPerspective(old_frame, matrix, (600, 280))
    old_gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
    p0 = np.array([[315,280],[315,265]], dtype="float32").reshape(-1,1,2) #pomits to track
    #print(p0)
    # Create a mask image for drawing purposes
    mask = np.zeros_like(res)
    while cap.isOpened():
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if ret == False:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        if frame.all()==None:
            break
        localize(frame,goodpts)
        #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        #cv.imshow('frame', gray)
        #if cv.waitKey(1) == ord('q'):
        #    break
    #rospy.Subscriber("image_topic", Image, start_up)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
goodpts = good_oldpts()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass