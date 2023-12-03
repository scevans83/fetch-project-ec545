#!/usr/bin/env python
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
# from sensor_msgs.msg import Image # Image is the message type
# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from std_msgs.msg import String
from cv_basics.msg import cv_results
from align_color import find_contours_and_colors, label_contours_and_colors
  
def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('/video_processing_results', cv_results, queue_size=10)
     
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
  # br = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()
         
      if ret == True:
        # Print debugging information to the terminal
        rospy.loginfo('processing image')

        desired_color = "red"
        red_lower = (0, 0, 175) 
        red_upper = (200, 200, 255)

        view_mask = True
        contours, color_list, mask = find_contours_and_colors(frame, red_lower, red_upper, lower = 3000, upper = 200000)
        
        if not view_mask:
          mask = None
        
        labeled_frame = label_contours_and_colors(frame, contours, color_list, mask = mask)


        cv2.imshow("test", labeled_frame)
        cv2.waitKey(10)

        pub_msg = cv_results()
        pub_msg.detected_color = desired_color if desired_color in color_list else ""
        pub.publish(pub_msg)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass