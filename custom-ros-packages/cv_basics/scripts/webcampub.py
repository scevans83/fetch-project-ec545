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
import numpy as np
from std_msgs.msg import String
from cv_basics.msg import cv_results
from fetch_controller.msg import state_status
from align_color import find_contours_and_colors, label_contours_and_colors, find_contour_center

curr_state = ""
desired_color = "red"

def state_status_callback(msg):
   global curr_state
   global desired_color

   rospy.loginfo("updating current state %s and desired color %s", msg.curr_state, msg.desired_color_detect)
   
   curr_state = msg.curr_state
   desired_color = msg.desired_color_detect
   
  
def publish_message():
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('/video_processing_results', cv_results, queue_size=10)
  rospy.Subscriber('/state_status', state_status, state_status_callback)
     
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
  last_offset = None
  global desired_color
  global curr_state
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()
         
      if ret == True:
        # Print debugging information to the terminal


        #select the proper mask
        if desired_color == "green":
          # lower_color_thresh = (0, 30, 0)
          # upper_color_thresh= (110, 150, 110)
          lower_color_thresh = (0, 0, 0)
          upper_color_thresh= (255, 255, 255)
          dist_thresh = 150
        elif desired_color == "red":
          lower_color_thresh = (0, 0, 175) 
          upper_color_thresh = (200, 200, 255)
          dist_thresh = 100
        else:
          lower_color_thresh = (0, 0, 0) 
          upper_color_thresh = (255, 255, 255)
           

        view_mask = True
        contours, color_list, mask = find_contours_and_colors(frame, lower_color_thresh, upper_color_thresh, lower = 3000, upper = 200000, color_dist_thresh=dist_thresh)
        
        if not view_mask:
          mask = None
        
        labeled_frame = label_contours_and_colors(frame, contours, color_list, mask = mask)


        cv2.imshow("test", labeled_frame)
        cv2.waitKey(10)

        curr_offset = None

        if desired_color in color_list:
          #find the max red contour
          max_color_idx = None
          max_color_area = -np.inf
          for idx in range(len(color_list)):
              if color_list[idx] == desired_color:
                  curr_area = cv2.contourArea(contours[idx])
                  if curr_area > max_color_area:
                      max_color_area = curr_area
                      max_color_idx = idx

          curr_offset = find_contour_center(contours[max_color_idx])[0]

        if(last_offset == None):
           last_offset = np.int32(np.shape(frame)[0]/2)

        if(curr_offset == None):
           curr_offset = np.int32(np.shape(frame)[0]/2)

        last_offset = curr_offset

        pub_msg = cv_results()
        pub_msg.detected_color = desired_color if desired_color in color_list else ""
        pub_msg.image_center = np.int32(np.shape(frame)[0]/2)
        pub_msg.contour_center = curr_offset if desired_color in color_list else last_offset
        pub.publish(pub_msg)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass