#!/usr/bin/env python
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
  
def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('video_frames', Image, queue_size=10)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  cap = cv2.VideoCapture(0)
     
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()
         
      if ret == True:
        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame')
             
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        pub.publish(br.cv2_to_imgmsg(frame))
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass


# #!/usr/bin/env python
# # coding:utf-8
# import math
# import cv2
# import numpy as np
# import time
# import random #added KO
# from common import *
# from time import sleep
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan, Image
# from dynamic_reconfigure.server import Server
# from cv_bridge import CvBridge
# from yahboomcar_laser.cfg import laserAvoidPIDConfig
# from align_color import find_contours_and_colors

# class display_Img:
#     def __init__(self):
#         rospy.on_shutdown(self.cancel)
#         self.r = rospy.Rate(20)
#         self.frame = cv2.VideoCapture(0)
#         self.pub = rospy.Publisher('video_frames', Image, queue_size=10)
#         self.br = CvBridge()
       
#     def cancel(self):
#         self.pub.unregister()
#         rospy.loginfo("Shutting down this node.")
        
#     def get_image(self):
        
#         br = CvBridge()
        
#         ret, img = self.frame.read()
#         if ret == true:
#             rospy.loginfo('publishing video frame')
#             self.pub.publish(br.cv2_to_imgmsg(img))
            
# if __name__ == '__main__':
#     rospy.init_node('video_pub_py', anonymous=True)
#     tracker = display_Img()
#     rospy.spin()
