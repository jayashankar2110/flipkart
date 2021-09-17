#!/usr/bin/env python
from numpy.lib.function_base import append
import rospy
import matplotlib.pyplot as plt
from single_bot.msg import localizemsg
import cv2 as cv
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from utils import ARUCO_DICT, aruco_display
robotvelocity  = []
msg=localizemsg()
pub=None
x_ar=[]
y_ar=[]
def find_angle(a,b,center_a,center_b):
    print(a,b)
    print(center_a,center_b)

    angle=math.atan2((b-center_b),(a-center_a))
    print(angle)
    #angle=angle+(math.pi/2)
    #if angle<0:
    #    angle=angle+2*math.pi

    #angledg=math.degrees(angle)
    return angle
def localize(data,image_pub):
    global robotvelocity
    global pub
    global msg
   
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "8UC3")
    

	# load the ArUCo dictionary, grab the ArUCo parameters, and detect
	# the markers
    #print("Detecting '{}' tags....".format('DICT_5X5_100'))
    arucoDict = cv.aruco.Dictionary_get(ARUCO_DICT['DICT_5X5_100'])
    arucoParams = cv.aruco.DetectorParameters_create()
    corners, ids, rejected = cv.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    #print(corners)
    flag=0
    if len(robotvelocity)==0:
        flag=1
    velocity=0
    detected_markers = aruco_display(corners, ids, rejected, image)
    
    rows=image.shape[0]
    cols=image.shape[1]

    for i in range(len(corners)):
        center_x=(corners[i][0][0][0] + corners[i][0][2][0]) //2
        center_y=(corners[i][0][1][1] + corners[i][0][3][1]) //2
        
        head_x=(corners[i][0][0][0] + corners[i][0][1][0]) //2
        head_y=(corners[i][0][0][1] + corners[i][0][1][1])//2
        detected_markers=cv.arrowedLine(detected_markers,(int(center_x),int(center_y)) , (int(head_x),int(head_y)),(0, 255, 255), 3)
        
        center_y=rows-center_y
        head_y=rows-head_y
        #print(rows)
        #print(center_y)
        angle=find_angle(head_x,head_y,center_x,center_y)

        msg.angle=angle
        x_cordinate=center_x/200 #calibrate to return in metters
        
        y_cordinate=center_y/225#calibrate to return in metters
        msg.x_cordinate=x_cordinate
        msg.y_cordinate=y_cordinate
        
        timestamp=float(rospy.get_time())
        msg.timestamp=timestamp
        msg.id=str(1)
        
        if flag!=1:
            for j in range(len(robotvelocity)):
                if robotvelocity[j][0]==ids[i]:
                    old_x=robotvelocity[j][1]
                    old_y=robotvelocity[j][2]
                    velocity=(((old_x-x_cordinate)**2+(old_y-y_cordinate)**2)**0.5)/(timestamp-robotvelocity[j][3]) # velocity control
                    robotvelocity[j][1]=x_cordinate
                    robotvelocity[j][2]=y_cordinate
                    robotvelocity[j][3]=float(rospy.get_time())
            
        else:
            robotvelocity.append([ids[i],x_cordinate,y_cordinate,timestamp])
        

        
        msg.velocity=velocity
        #rospy.loginfo(msg)
        x_ar.append(x_cordinate)
        y_ar.append(y_cordinate)
        pub.publish(msg)
    #print(robotvelocity,ids)
    if len(corners)==0:   #if none are detected
        msg.x_cordinate=0
        msg.y_cordinate=0
        timestamp=float(rospy.get_time())
        msg.timestamp=timestamp
        msg.id=str(0)
        msg.velocity=0
    realtime=str(rospy.get_time())
    image = cv.putText(detected_markers, realtime, (50,50 ), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2, cv.LINE_AA)
    ros_img=bridge.cv2_to_imgmsg(image, "passthrough")
    image_pub.publish(ros_img)
    #cv.imshow('test',image) #debug output
    #cv.waitKey(10)
    #cv.imshow('test',detected_markers) debug output
    #cv.waitKey(20)
    



    


    
    

    
def talker():
   
        
    # Apply Perspective Transform Algorithm
    #Caliibratation code
    #matrix = cv.getPerspectiveTransform(pts1, pts2)
    #result = cv.warpPerspective(old_frame, matrix, (600, 600))
    #res = cv.warpPerspective(old_frame, matrix, (600, 280))

      
    global pub
    rospy.init_node('localzation')
    image_pub = rospy.Publisher("image_feedback", Image,queue_size=5)
    pub = rospy.Publisher('/feedback', localizemsg, queue_size=10)
    rospy.Subscriber("image_topic", Image, localize,image_pub)
   
    rospy.spin()
    

if __name__ == '__main__':
    try:
        talker()
        #x_ar=np.array(x_ar)
        #print(x_ar)
        #x_ar=np.diff(x_ar)
        #m=np.mean(x_ar)
        #print(m)
        #print(y_ar)
    except rospy.ROSInterruptException:
        pass
