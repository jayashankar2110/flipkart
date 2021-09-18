#!/usr/bin/env python3


# Python libs
import requests

# numpy and scipy
import numpy as np


# OpenCV
import cv2
import pdb
# Ros libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# Ros Messages
#from sensor_msgs.msg import CompressedImage

image_pub = rospy.Publisher("image_topic", Image,queue_size=5)
rospy.init_node('image', anonymous=True)
url = "http://192.168.0.102:8080/shot.jpg"
#cv2.namedWindow("Keypoints",cv2.WINDOW_NORMAL)
bridge = CvBridge()

# While loop to continuously fetching data from the Url
while not rospy.is_shutdown():
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    im = cv2.imdecode(img_arr, -1)
    #pts1 = np.float32([[150, 60], [1650,0], [200, 875], [1650, 875]])
    #pts2 = np.float32([[0, 0], [1500,0], [0,780], [1500, 780]])
    #pts1 = np.float32([[130, 195], [1550,132], [120, 938], [1580, 938]])
    #pts2 = np.float32([[0, 0], [1496,0], [0,780], [1496, 780]])
    #pts1 = np.float32([[127, 201], [1560,94], [175,950 ], [1570, 910]])
    #pts2 = np.float32([[0, 0], [1496,0], [0,780], [1496, 780]])
    #pts1 = np.float32([[0, 230], [1600,120], [70,1045], [1620, 920]])
    pts1 = np.float32([[10, 94], [600,100], [10,395], [600, 395]])
    pts2 = np.float32([[0, 0], [1496,0], [0,780], [1496, 780]])
    # Apply Perspective Transform Algorithm
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    im = cv2.warpPerspective(im, matrix, (1550, 830))
    #res1 = cv.warpPerspective(frame, matrix, (600, 280))
    scale_percent = 100 # percent of original size
    width = int(im.shape[1] * scale_percent / 100)
    height = int(im.shape[0] * scale_percent / 100)
    


    im = cv2.resize(im,(width,height))
    #im = cv2.rotate(im, cv2.ROTATE_180)
   
    rows=im.shape[0]
    cols=im.shape[1]
    
    M = cv2.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),180,1)
    dst = cv2.warpAffine(im,M,(cols,rows))
    ros_img=bridge.cv2_to_imgmsg(dst, "passthrough")
    
    image_pub.publish(ros_img)
    
  
#cv2.destroyAllWindows()
