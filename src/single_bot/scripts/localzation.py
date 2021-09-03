#!/usr/bin/env python
import rospy
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
def find_angle(a,b,center_a,center_b):
    angle=math.atan2((center_a-a),(center_b-b))
    if angle<0:
        angle=angle+2*math.pi

    angle=math.degrees(angle)
    return angle
def localize(data):
    global robotvelocity
    global pub
    global msg
   
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "8UC3")
    

	# load the ArUCo dictionary, grab the ArUCo parameters, and detect
	# the markers
    print("Detecting '{}' tags....".format('DICT_5X5_100'))
    arucoDict = cv.aruco.Dictionary_get(ARUCO_DICT['DICT_5X5_100'])
    arucoParams = cv.aruco.DetectorParameters_create()
    corners, ids, rejected = cv.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    #print(corners)
    flag=0
    if len(robotvelocity)==0:
        flag=1
    velocity=0
    detected_markers = aruco_display(corners, ids, rejected, image)
    for i in range(len(corners)):
        center_x=(corners[i][0][0][0] + corners[i][0][2][0]) //2
        center_y=(corners[i][0][1][1] + corners[i][0][3][1]) //2
        head_x=(corners[i][0][0][0] + corners[i][0][1][0]) //2
        head_y=(corners[i][0][0][1] + corners[i][0][1][1])//2
        detected_markers=cv.arrowedLine(detected_markers,(int(center_x),int(center_y)) , (int(head_x),int(head_y)),(0, 255, 255), 3)
        angle=find_angle(head_x,head_y,center_x,center_y)
        msg.angle=angle
        x_cordinate=center_x//31.5 #calibrate to return in metters
        y_cordinate=center_y//27 #calibrate to return in metters
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
    cv.imshow('test',image) #debug output
    cv.waitKey(10)
    #cv.imshow('test',detected_markers) debug output
    #cv.waitKey(20)
    



    


    
    

    
def talker():
   
        
    # Apply Perspective Transform Algorithm
    #Caliibratation code
    #matrix = cv.getPerspectiveTransform(pts1, pts2)
    #result = cv.warpPerspective(old_frame, matrix, (600, 600))
    #res = cv.warpPerspective(old_frame, matrix, (600, 280))
    
    global pub
    rospy.init_node('localzation', anonymous=True)

    rospy.Subscriber("image_topic", Image, localize)
    pub = rospy.Publisher('/feedback', localizemsg, queue_size=10)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass