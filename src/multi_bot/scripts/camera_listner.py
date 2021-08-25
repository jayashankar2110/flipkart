#!/usr/bin/env python


# Python libs
import requests

# numpy and scipy
import numpy as np


# OpenCV
import cv2

# Ros libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# Ros Messages
#from sensor_msgs.msg import CompressedImage

image_pub = rospy.Publisher("image_topic", Image,queue_size=5)
rospy.init_node('image', anonymous=True)
url = "http://192.168.0.236:8080/shot.jpg"
#cv2.namedWindow("Keypoints",cv2.WINDOW_NORMAL)
bridge = CvBridge()

# While loop to continuously fetching data from the Url
while not rospy.is_shutdown():
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    im = cv2.imdecode(img_arr, -1)
    scale_percent = 60 # percent of original size
    width = int(im.shape[1] * scale_percent / 100)
    height = int(im.shape[0] * scale_percent / 100)

    im = cv2.resize(im,(width,height))
    ros_img=bridge.cv2_to_imgmsg(im, "passthrough")
    image_pub.publish(ros_img)
    
  
#cv2.destroyAllWindows()